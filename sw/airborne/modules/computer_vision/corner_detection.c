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
#include "state.h"
#include "subsystems/datalink/telemetry.h"

#define IMG_WIDTH 272
#define IMG_HEIGHT 272

// THIS NEEDS TO BE MODIFIED.

#define FOV_W 2.968253968
#define FOV_H 2.968253968

// FAST SETTINGS
bool_t fast_show_features = FALSE;
uint8_t fast_threshold = 5;
uint16_t fast_min_dist  = 10;
uint16_t fast_x_padding = 110;
uint16_t fast_y_padding = 50;

// LK SETTINGS
bool_t lk_show_optical_flow = TRUE;
uint16_t lk_half_window_size = 10;
uint16_t lk_subpixel_factor = 10;
uint8_t lk_max_iterations = 10;
uint8_t lk_step_threshold = 2;
uint16_t lk_max_points = 50;

// IMG = 272 x 272
struct image_t img_gray;
struct image_t img_old;

// Declare the state of the aircraft
struct state_t prev_state;
struct state_t temp_state;

// declare utility functions
struct point_t* _init_grid(int width, int height);
void _draw_line(struct image_t *img, int x);

// Featureless iteration
uint8_t Featurless = FALSE;

// Result
float divergence;

void corner_detection_init(void)
{
  // Initialize images
  image_create(&img_gray, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);
  image_create(&img_old, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);

  // Initialize current state
  temp_state.phi   = 0;
  temp_state.theta = 0;

  // Initialize previous state
  prev_state.phi   = 0.0;
  prev_state.theta = 0.0;

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
  // REMEMBER THE POSSIBILITY OF INCLUDING AN ADAPTIVE THRESHOLD
  struct point_t *features = fast9_detect(
          &img_gray,
          fast_threshold,
          fast_min_dist,
          fast_x_padding,
          fast_y_padding,
          &feature_cnt);

  // Decrease and increase the threshold based on previous values
  if (feature_cnt < 40 && fast_threshold > 5) {
    fast_threshold--;
  } else if (feature_cnt > 50 && fast_threshold < 60) {
    fast_threshold++;
  }

  // Check if we found some corners to track
  Featurless = FALSE;
  if (feature_cnt < 2) { // Set this threshold according to the features detected in the CZ
      Featurless = TRUE;
      free(features);
      image_copy(&img_gray, &img_old);
      divergence = 0;
      DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel, DefaultDevice, &feature_cnt, &divergence, &fast_threshold);
      return TRUE;
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

  for (int i = 0; i <feature_cnt ; ++i) {
    if (vectors[i].pos.y > (IMG_HEIGHT / 2) * lk_subpixel_factor) {
      vectors[i].flow_x = 0;
      vectors[i].flow_y = 0;
    }
  }

  // Show optical flow on original image
  if (lk_show_optical_flow)
    image_show_flow(img, vectors, feature_cnt, lk_subpixel_factor);

//   Derotate the flow
  temp_state.phi   = stateGetNedToBodyEulers_f()->phi;
  temp_state.theta = stateGetNedToBodyEulers_f()->theta;

  float diff_flow_x = (temp_state.phi - prev_state.phi) * IMG_WIDTH / FOV_W;
  float diff_flow_y = (temp_state.theta - prev_state.theta) * IMG_HEIGHT / FOV_H;

  for (int i = 0; i <feature_cnt ; ++i) {
      vectors[i].flow_x = vectors[i].flow_x - diff_flow_x * 10;
      vectors[i].flow_y = vectors[i].flow_y - diff_flow_y * 10;
  }

  // Analyze the flow
  float error_threshold = 10.0f;
  int n_iterations_RANSAC = 20;
  int n_samples_RANSAC = 5;
  struct linear_flow_fit_info fit_info;

  int success_fit = analyze_linear_flow_field(vectors, feature_cnt, error_threshold, n_iterations_RANSAC, n_samples_RANSAC, IMG_WIDTH, IMG_HEIGHT, &fit_info);

  if (!success_fit) {
    fit_info.divergence = 0.0f;
  }

  // Result
  divergence = fit_info.divergence * 10;

  // Copy new image to old image
  image_copy(&img_gray, &img_old);

  DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel, DefaultDevice, &feature_cnt, &divergence, &fast_threshold);

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


