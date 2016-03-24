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

#define MEMORY 5

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
uint16_t lk_max_points       = 100;

// IMG = 272 x 272
struct image_t img_gray;
struct image_t img_old;

// Rates
float yaw_rate;

// BALANCE
struct flow regions[4];

// FILTER NOISE
int threshold_noise = 50;
float filter_previous_average_flow[4];
float filter_previous_counter[4];
float alpha = 0.5;
float alpha_counter = 0.5;

// FIRST CONDITION: Check if the aircraft is turning
float threshold_turning = 0.025;
uint8_t TURNING = FALSE;
float turning[MEMORY];
int counter_turning;

void corner_detection_init(void)
{
  // Initialize images
  image_create(&img_gray, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);
  image_create(&img_old, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);

  // Initialize previous filter value
  filter_previous_average_flow[0] = 0;
  filter_previous_average_flow[1] = 0;
  filter_previous_average_flow[2] = 0;
  filter_previous_average_flow[3] = 0;

  filter_previous_counter[0] = 0;
  filter_previous_counter[1] = 0;
  filter_previous_counter[2] = 0;
  filter_previous_counter[3] = 0;

  // Initialize the variables related with turning
  TURNING = FALSE;
  turning[0] = 0;
  turning[1] = 0;
  turning[2] = 0;
  turning[3] = 0;

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

  // Show optical flow on original image
  if (lk_show_optical_flow)
    image_show_flow(img, vectors, feature_cnt, lk_subpixel_factor);

  // Narrow the vertical window and only horizontal derotated flow
  yaw_rate = stateGetBodyRates_f()->r;

  // Initialize regions
  for (int i = 0; i < 4; ++i) {
    regions[i].counter = 0;
    regions[i].total = 0;
    regions[i].average = 0;
  }

  // Loop over all flow features
  for (int i = 0; i < feature_cnt; ++i) {
    // Obtain image coordinates
    int x = vectors[i].pos.x / lk_subpixel_factor;
    int y = vectors[i].pos.y / lk_subpixel_factor;

    // Ignore any flow feature below a certain horizontal
    if (y > IMG_HEIGHT / 2)
      continue;

    // De-rotate flow
    vectors[i].flow_x -= (y - IMG_HEIGHT / 2) * yaw_rate;

    // Determine region
    int k = x * 4 / IMG_WIDTH;

    regions[k].counter += 1;
    regions[k].total += abs(vectors[i].flow_x); // Not to sure about this??
  }

  // Calculate average of flow regions
  for (int i = 0; i < 4; ++i) {
    // If region counter is greater than zero, calculate average
    if (regions[i].counter > 0) {
      regions[i].average = regions[i].total / regions[i].counter;
    }

    // If the average is higher than the threshold, then we do not use it
    if (regions[i].average > threshold_noise) {
      regions[i].average = threshold_noise;
    }

    // Filters
    regions[i].average = alpha * regions[i].average + (1-alpha) * filter_previous_average_flow[i];
    regions[i].counter = alpha_counter * regions[i].counter + (1-alpha_counter) * filter_previous_counter[i];
  }

  // Update current values
  turning[4] = yaw_rate;

  // Check turning
  counter_turning = 0;
  for (int i = 0; i <MEMORY ; ++i) {
    if (abs(turning[i]) > threshold_turning) {
      counter_turning++;
    }
  }
  TURNING = 0;
  if (counter_turning > 0) {
    TURNING = 1;
    for (int i = 0; i <4 ; ++i) {
      regions[i].average = 0;
    }
  }

  DOWNLINK_SEND_OPTICAL_FLOW(DefaultChannel,
                             DefaultDevice,
                             &regions[3].counter,
                             &regions[2].counter,
                             &regions[1].counter,
                             &regions[0].counter,
                             &regions[3].average,
                             &regions[2].average,
                             &regions[1].average,
                             &regions[0].average);

  // Copy new image to old image
  image_copy(&img_gray, &img_old);

  // Prepare variables for the next iteration
  turning[0] = turning[1];
  turning[1] = turning[2];
  turning[2] = turning[3];
  turning[3] = turning[4];

  // Copy flow values
  filter_previous_average_flow[0] = regions[0].average;
  filter_previous_average_flow[1] = regions[1].average;
  filter_previous_average_flow[2] = regions[2].average;
  filter_previous_average_flow[3] = regions[3].average;

  filter_previous_counter[0] = regions[0].counter;
  filter_previous_counter[1] = regions[1].counter;
  filter_previous_counter[2] = regions[2].counter;
  filter_previous_counter[3] = regions[3].counter;

  // Free memory
  free(features); features = NULL;
  free(vectors); vectors = NULL;

  return TRUE;
}