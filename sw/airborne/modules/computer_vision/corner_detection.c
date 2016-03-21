/*
 * Copyright (C) Mavlabcourse 2016 - Group 05
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/corner_detection.c
 */

// Own header
#include "modules/computer_vision/corner_detection.h"
#include "lib/vision/fast_rosten.h"
#include "lib/vision/lucas_kanade.h"
#include "modules/computer_vision/opticflow/linear_flow_fit.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include <math.h>

#define IMG_WIDTH 272
#define IMG_HEIGHT 272

// FAST SETTINGS
bool_t fast_show_features = FALSE;
uint8_t fast_threshold    = 5;
uint16_t fast_min_dist    = 10;
uint16_t fast_x_padding   = 5;
uint16_t fast_y_padding   = 50;

// Storage for feature & vector count
uint16_t feature_cnt;

// LK SETTINGS
bool_t lk_show_optical_flow  = FALSE;
uint16_t lk_half_window_size = 10;
uint16_t lk_subpixel_factor  = 10;
uint8_t lk_max_iterations    = 10;
uint8_t lk_step_threshold    = 2;
uint16_t lk_max_points       = 50;

// IMG = 272 x 272
struct image_t img_gray;
struct image_t img_old;

// FEATURELESS ITERATION
uint8_t Featureless = FALSE;

// BALANCE
int counter_right_far;
int counter_left_far;
int counter_right_close;
int counter_left_close;

float flow_right_far;
float flow_left_far;
float flow_right_close;
float flow_left_close;

float med_flow_right_far;
float med_flow_left_far;
float med_flow_right_close;
float med_flow_left_close;

void corner_detection_init(void)
{
  // Initialize images
  image_create(&img_gray, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);
  image_create(&img_old, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);

  // Add detection function to CV
  cv_add(corner_detection_func);
}

bool_t corner_detection_func(struct image_t* img)
{
  // Convert image to grayscale
  image_to_grayscale(img, &img_gray);

  // Find features to track
  struct point_t *features = fast9_detect(
          &img_gray,
          fast_threshold,
          fast_min_dist,
          fast_x_padding,
          fast_y_padding,
          &feature_cnt);

  // Adaptive threshold
  if (feature_cnt < 40 && fast_threshold > 5) {
    fast_threshold--;
  } else if (feature_cnt > 50 && fast_threshold < 60) {
    fast_threshold++;
  }

  // Show fast features found
  if (fast_show_features)
    image_show_points(img, features, feature_cnt);

  // Calculate optical flow from features found
  struct flow_t *vectors = opticFlowLK(
          &img_gray,
          &img_old,
          features,
          &feature_cnt,
          lk_half_window_size,
          lk_subpixel_factor,
          lk_max_iterations,
          lk_step_threshold,
          lk_max_points);

  // Narrow the vertical window and only horizontal derotated flow
  yaw_rate = stateGetBodyRates_f()->r;

  for (int i = 0; i <feature_cnt ; ++i) {
    if (vectors[i].pos.y > (IMG_HEIGHT / 2) * lk_subpixel_factor) {
      vectors[i].flow_x = 0;
      vectors[i].flow_y = 0;
    } else {
      vectors[i].flow_x = vectors[i].flow_x + ((IMG_HEIGHT/2) - (vectors[i].pos.y/lk_subpixel_factor)) * yaw_rate;
      vectors[i].flow_y = 0;
    }
  }

  // Determine the number of features and flow in each region (4 REGIONS)
  counter_right_far   = 0;
  counter_left_far    = 0;
  counter_right_close = 0;
  counter_left_close  = 0;

  flow_right_far   = 0;
  flow_left_far    = 0;
  flow_right_close = 0;
  flow_left_close  = 0;

  for (int i = 0; i <feature_cnt ; ++i) {
    if (vectors[i].pos.y < (IMG_HEIGHT / 2) * lk_subpixel_factor) {
      if (vectors[i].pos.x > (3 * IMG_WIDTH / 4) * lk_subpixel_factor) {
        counter_right_far++;
        flow_right_far = abs(flow_right_far) + vectors[i].flow_x;
      } else if (vectors[i].pos.x > (IMG_WIDTH / 2) * lk_subpixel_factor && vectors[i].pos.x < (3 * IMG_WIDTH / 4) * lk_subpixel_factor) {
        counter_right_close++;
        flow_right_close = abs(flow_right_close) + vectors[i].flow_x;
      } else if (vectors[i].pos.x > (IMG_WIDTH / 4) * lk_subpixel_factor && vectors[i].pos.x < (IMG_WIDTH / 2) * lk_subpixel_factor) {
        counter_left_close++;
        flow_left_close = abs(flow_left_close) + vectors[i].flow_x;
      } else if (vectors[i].pos.x < (IMG_WIDTH / 4) * lk_subpixel_factor) {
        counter_left_far++;
        flow_left_far = abs(flow_left_far) + vectors[i].flow_x;
      }
    }
  }

  // Medium flow in each region
  med_flow_right_far   = 0;
  med_flow_left_far    = 0;
  med_flow_right_close = 0;
  med_flow_left_close  = 0;

  if (counter_right_far >0) {
    med_flow_right_far = flow_right_far / counter_right_far;
  } else {
    med_flow_right_far = 0;
  }

  if (counter_right_close >0) {
    med_flow_right_close = flow_right_close / counter_right_close;
  } else {
    med_flow_right_close = 0;
  }

  if (counter_left_close >0) {
    med_flow_left_close = flow_left_close / counter_left_close;
  } else {
    med_flow_left_close = 0;
  }

  if (counter_left_far >0) {
    med_flow_left_far = flow_left_far / counter_left_far;
  } else {
    med_flow_left_far = 0;
  }

  DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel, DefaultDevice, &counter_right_far, &counter_right_close, &counter_left_far, &counter_left_close, &med_flow_right_far, &med_flow_right_close, &med_flow_left_far, &med_flow_left_close);

  // Copy new image to old image
  image_copy(&img_gray, &img_old);

  // Free memory
  free(features); features = NULL;
  free(vectors); vectors = NULL;

  return TRUE;
}