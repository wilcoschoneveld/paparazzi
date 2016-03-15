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

// Result
uint16_t corner_cnt     = 0;
int      corner_cnt_int = 0; // Counter sent to Messages in Paparazzi

// Thresholds FAST
uint8_t threshold  = 20;
uint16_t min_dist  = 10;
uint16_t x_padding = 0; // The padding in the x direction to not scan for corners
uint16_t y_padding = 0; // The padding in the x direction to not scan for corners

// Function
bool_t corner_detection_func(struct image_t* img);
bool_t corner_detection_func(struct image_t* img)
{

  // Corner detection
  struct point_t *corners = fast9_detect(img, threshold, min_dist, x_padding, y_padding, &corner_cnt);

  corner_cnt_int = corner_cnt;


//////////////////////////////////////////////////////////////////////////////////

//  uint8_t *source = img->buf;
//  uint8_t *dest   = img->buf;
//  int flag        = 0;
//  int height      = img->h;
//  int width       = img->w;
//
//  // Display the corners in the image
//  for (int row=0; row <height; row++){
//	  for (int col=0; col <width; col++){
//
//		  // Check if the pixel is defined as a corner
//		  int position=0;
//		  while (position <corner_cnt && flag == 0){
//
//			  // If a corner is detected, then color red.
//			  if (corners[position].x == col && corners[position].y == row){
//				  flag = 1;
//				  // UYVY
//				  dest[0] = 64;         // U
//				  dest[1] = source[1];  // Y
//				  dest[2] = 255;        // V
//				  dest[3] = source[3];  // Y
//			  }
//			  position++;
//		}
//
//		// If the pixel does not contain a corner, then greyscale
//		if (flag == 0){
//			// UYVY
//			char u = source[0] - 127;
//			u /= 4;
//			dest[0] = 127;        // U
//			dest[1] = source[1];  // Y
//			u = source[2] - 127;
//			u /= 4;
//			dest[2] = 127;        // V
//			dest[3] = source[3];  // Y
//		}
//
//		// Go to the next 2 pixels
//		dest   += 4;
//		source += 4;
//		flag = 0;
//	  }
//  }

////////////////////////////////////////////////////////////////////////////////
  
  DOWNLINK_SEND_COLORFILTER(DefaultChannel, DefaultDevice, &corner_cnt_int);
  return FALSE;
}

void corner_detection_init(void)
{
  cv_add(corner_detection_func);
}

