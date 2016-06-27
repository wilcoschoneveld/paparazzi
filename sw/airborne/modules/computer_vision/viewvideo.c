/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"
#include "modules/computer_vision/cv.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include <pthread.h>
#include "mcu_periph/sys_time.h"

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

#include BOARD_CONFIG


// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Default image folder
#ifndef VIEWVIDEO_SHOT_PATH
#ifdef VIDEO_THREAD_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH VIDEO_THREAD_SHOT_PATH
#else
#define VIEWVIDEO_SHOT_PATH /data/video/images
#endif
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 10
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

struct image_t img_copy;
pthread_mutex_t img_mutex;
pthread_cond_t img_available_cv;

/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
struct image_t *viewvideo_function(struct image_t *img);
struct image_t *viewvideo_function(struct image_t *img) {

  // If image is not yet processed by thread, return
  if (pthread_mutex_trylock(&img_mutex) != 0 || viewvideo.new_image) {
    return NULL;
  }

  // If the image buffer has not yet been initialized
  if (img_copy.buf_size == 0) {
    // Create a new image based on downsize factor
    image_create(&img_copy,
                 img->w / viewvideo.downsize_factor,
                 img->h / viewvideo.downsize_factor,
                 IMAGE_YUV422);
  }

  // Copy image with downsize factor (1 does a direct copy)
  image_yuv422_downsample(img, &img_copy, viewvideo.downsize_factor);

  // Inform thread of new image
  viewvideo.new_image = true;
  pthread_cond_signal(&img_available_cv);
  pthread_mutex_unlock(&img_mutex);

  return NULL;
}


void viewvideo_send_frame(void);
void viewvideo_send_frame(void) {

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, img_copy.w, img_copy.h, IMAGE_JPEG);

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc %s %d 2>/dev/null", STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT);
#endif

  if (viewvideo.is_streaming) {

    jpeg_encode_image(&img_copy, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);

#if VIEWVIDEO_USE_NETCAT
    // Open process to send using netcat (in a fork because sometimes kills itself???)
    pid_t pid = fork();

    if (pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    } else if (pid == 0) {
      // We are the child and want to send the image
      FILE *netcat = popen(nc_cmd, "w");
      if (netcat != NULL) {
        fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process.\n");
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    } else {
      // We want to wait until the child is finished
      wait(NULL);
    }
#else

    if (viewvideo.use_rtp) {

      // Send image with RTP
      rtp_frame_send(
        &video_sock,              // UDP socket
        &img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        VIEWVIDEO_RTP_TIME_INC    // 90kHz time increment
      );
      // Extra note: when the time increment is set to 0,
      // it is automaticaly calculated by the send_rtp_frame function
      // based on gettimeofday value. This seems to introduce some lag or jitter.
      // An other way is to compute the time increment and set the correct value.
      // It seems that a lower value is also working (when the frame is received
      // the timestamp is always "late" so the frame is displayed immediately).
      // Here, we set the time increment to the lowest possible value
      // (1 = 1/90000 s) which is probably stupid but is actually working.
    }
#endif
  }

  // Free all buffers
  image_free(&img_jpeg);
}


void *viewvideo_thread(void *args);
void *viewvideo_thread(void *args) {
  // Time-keeping variables
  struct timespec time_now, time_prev;
  clock_gettime(CLOCK_MONOTONIC, &time_prev);
  int32_t delta = 0;
  uint32_t fps_period = 1000000 / VIEWVIDEO_FPS;

  // Request new image from video thread
  pthread_mutex_lock(&img_mutex);
  viewvideo.new_image = false;

  while (viewvideo.is_streaming) {
    // Wait for img available signal
    pthread_cond_wait(&img_available_cv, &img_mutex);

    // If there is no image available, try again
    if (!viewvideo.new_image) {
      continue;
    }

    // Send a new frame from this thread
    viewvideo_send_frame();

    // Obtain current time
    clock_gettime(CLOCK_MONOTONIC, &time_now);

    // Increase delta with desired frame rate minus elapsed time
    delta += fps_period - sys_time_elapsed_us(&time_prev, &time_now);

    // If delta is positive, this thread is faster than required
    if (delta > 0) {
      // Sleep for delta time to maintain desired frame rate
      usleep(delta);
    } else {
      // Diminish delta penalty to smooth out frame rate
      delta /= 2;
    }

    // Store timestamp as previous
    time_prev = time_now;

    // Request new image
    viewvideo.new_image = false;
  }

  pthread_mutex_unlock(&img_mutex);

  return NULL;
}


/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  char save_name[512];

  img_copy.buf_size = 0;  // Explicitly mark img_copy as uninitialized
  cv_add_to_device(&VIEWVIDEO_CAMERA, viewvideo_function);

  viewvideo.is_streaming = true;

#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "%s/netcat-recv.sh", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create netcat receiver file.\n");
  }
#else
  // Open udp socket
  udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);

  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create SDP file.\n");
  }
#endif

  // Initialize mutex and condition variable
  pthread_mutex_init(&img_mutex, NULL);
  pthread_cond_init(&img_available_cv, NULL);

  // Create new viewvideo thread
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, viewvideo_thread, NULL);
}