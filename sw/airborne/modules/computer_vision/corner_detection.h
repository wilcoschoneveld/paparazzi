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
 * @file modules/computer_vision/corner_detection.h
 */

#ifndef CORNER_DETECTION_CV_PLUGIN_H
#define CORNER_DETECTION_CV_PLUGIN_H

#include <stdint.h>

#include "modules/computer_vision/cv.h"

// Module settings
extern uint8_t fast_threshold;

// Module functions
void corner_detection_init(void);
bool_t corner_detection_func(struct image_t* img);

#endif
