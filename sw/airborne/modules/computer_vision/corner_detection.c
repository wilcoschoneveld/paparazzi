/*
 * Copyright (C) 2015
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

#include "subsystems/datalink/telemetry.h"

#define IMG_WIDTH 272
#define IMG_HEIGHT 272

// FAST SETTINGS
bool_t fast_show_features = FALSE;
uint8_t fast_threshold = 10;
uint16_t fast_min_dist  = 10;
uint16_t fast_x_padding = 100;
uint16_t fast_y_padding = 50;

// LK SETTINGS
bool_t lk_show_optical_flow = TRUE;
uint16_t lk_half_window_size = 10;
uint16_t lk_subpixel_factor = 10;
uint8_t lk_max_iterations = 10;
uint8_t lk_step_threshold = 2;
uint16_t lk_max_points = POINTS_X * POINTS_Y;

// IMG = 272 x 272
struct image_t img_gray;
struct image_t img_old;

// declare utility functions
struct point_t* _init_grid(int width, int height);
void _draw_line(struct image_t *img, int x);


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
  // Storage for feature & vector count
  uint16_t feature_cnt;

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

  // Show optical flow on original image
  if (lk_show_optical_flow)
    image_show_flow(img, vectors, feature_cnt, lk_subpixel_factor);

  // Analyze the flow
  float error_threshold = 10.0f;
  int n_iterations_RANSAC = 20;
  int n_samples_RANSAC = 5;
  struct linear_flow_fit_info fit_info;

  int success_fit = analyze_linear_flow_field(vectors, feature_cnt, error_threshold, n_iterations_RANSAC, n_samples_RANSAC, IMG_WIDTH, IMG_HEIGHT, &fit_info);

  if (!success_fit) {
    fit_info.divergence = 0.0f;
  }

  float divergence = fit_info.divergence;

  // Copy new image to old image
  image_copy(&img_gray, &img_old);

  DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel, DefaultDevice, &divergence);

  // Free memory
  free(features); features = NULL;
  free(vectors); vectors = NULL;

  return TRUE;
}



void _draw_line(struct image_t* img, int x) {

  struct point_t a, b;

  a.x = x;
  a.y = 0;
  b.x = x;
  b.y = IMG_HEIGHT;

  image_draw_line(img, &a, &b);
}


struct point_t* _init_grid(int width, int height) {
  int num_points = width * height;

  struct point_t* points = malloc(sizeof(struct point_t) * num_points);

  int x_spacing = (IMG_WIDTH - 2 * fast_x_padding) / (width - 1);
  int y_spacing = (IMG_HEIGHT - 2 * fast_y_padding) / (height - 1);

  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      int idx = j * width + i;
      points[idx].x = fast_x_padding + x_spacing * i;
      points[idx].y = fast_y_padding + y_spacing * j;
    }
  }

  return points;
}


