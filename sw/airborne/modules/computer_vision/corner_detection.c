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
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/corner_detection.h"
#include "lib/vision/fast_rosten.h"

#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>

// Thresholds FAST
uint8_t threshold  = 20;
uint16_t min_dist  = 10;
uint16_t x_padding = 30; // The padding in the x direction to not scan for corners
uint16_t y_padding = 60; // The padding in the y direction to not scan for corners

// Function
bool_t corner_detection_func(struct image_t* img);
bool_t corner_detection_func(struct image_t* img)
{
  uint16_t feature_cnt;

  // IMG = 272 x 272
  image_to_grayscale(img, img);

  struct point_t *features = fast9_detect(img, threshold, min_dist, x_padding, y_padding, &feature_cnt);

  image_show_points(img, features, feature_cnt);

  return FALSE;
}

void corner_detection_init(void)
{
  cv_add(corner_detection_func);
}

