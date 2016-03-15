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

#include "subsystems/datalink/telemetry.h"

#define IMG_WIDTH 272
#define IMG_HEIGHT 272


// Thresholds FAST
uint8_t threshold  = 20;
uint16_t min_dist  = 10;
uint16_t x_padding = 30; // The padding in the x direction to not scan for corners
uint16_t y_padding = 100; // The padding in the y direction to not scan for corners

// IMG = 272 x 272
struct image_t img_gray;
struct image_t img_old;

// Bad way to define but who cares
#define POINTS_X 10
#define POINTS_Y 10

struct point_t grid[POINTS_X * POINTS_Y];


void corner_detection_init(void)
{
  // Initialize grid
  int x_spacing = (IMG_WIDTH - 2 * x_padding) / (POINTS_X - 1);
  int y_spacing = (IMG_HEIGHT - 2 * y_padding) / (POINTS_Y - 1);

  for (int i = 0; i < POINTS_X; ++i) {
    for (int j = 0; j < POINTS_Y; ++j) {
      int idx = j * POINTS_X + i;
      grid[idx].x = x_padding + x_spacing * i;
      grid[idx].y = y_padding + y_spacing * j;
    }
  }

  // Initialize images
  image_create(&img_gray, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);
  image_create(&img_old, IMG_WIDTH, IMG_HEIGHT, IMAGE_GRAYSCALE);

  // Add detection function to CV
  cv_add(corner_detection_func);
}

bool_t corner_detection_func(struct image_t* img)
{
  // Storage for feature & vector count
  uint16_t feature_cnt = POINTS_X * POINTS_Y;

  // Convert image to grayscale
  image_to_grayscale(img, &img_gray);

  // Find features to track
//  struct point_t *features = fast9_detect(&img_gray, threshold, min_dist, x_padding, y_padding, &feature_cnt);

  // Show fast features found
//  image_show_points(img, features, feature_cnt);

  // Calculate optical flow from features found
  struct flow_t *vectors = opticFlowLK(&img_gray, &img_old, grid, &feature_cnt, 10, 10, 10, 2, POINTS_X * POINTS_Y);

  // Show optical flow on original image
  image_show_flow(img, vectors, feature_cnt, 10);

  // Free vector memory
  free(vectors);

  // Copy new image to old image
  image_copy(&img_gray, &img_old);

  // TODO: why false?
  return TRUE;
}

