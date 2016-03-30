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
 * @file "modules/obstacle_avoider/obstacle_avoider.c"
 * @author Group 05
 * Set of functions used for autonomous flight
 */

#include "modules/obstacle_avoider/obstacle_avoider.h"
#include "modules/computer_vision/corner_detection.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>
#include "subsystems/datalink/telemetry.h"
#include <math.h>

#define MEMORY_FRONTAL 3

// SECOND CONDITION: Check featureless regions
int featureless_indicator[4];

uint8_t FEATURELESS = FALSE;

float threshold_feature_far   = 0; // NOT USED
float threshold_feature_close = 4.1;

float changeHeading_Featureless_Far   = 20; // NOT USED
float changeHeading_Featureless_Close = 120;

// THIRD CONDITION: Check frontal obstacle
uint8_t FRONTAL_OBSTACLE = FALSE;

float frontal_threshold = 8;

float average_flow_LC[MEMORY_FRONTAL];
float average_flow_RC[MEMORY_FRONTAL];

int counter_flow_LC;
int counter_flow_RC;

float changeHeading_OF_Frontal = 180;
float changeHeading_OF_Lateral = 90;

// FOURTH CONDITION: Check side obstacles
uint8_t SIDE_OBSTACLE = FALSE; // NOT USED

float sideFar_threshold = 10; // NOT USED
float changeHeading_OF_sideFar = 15; // NOT USED

// Change heading
float changeHeading_amount_Outside     = 0;
float changeHeading_amount_Featureless = 0;
float changeHeading_amount_Frontal     = 0;
float changeHeading_amount_Side        = 0;

void obstacle_avoider_init() {

	// Initialize variables
	for (int i = 0; i <(MEMORY_FRONTAL-1); ++i) {
		average_flow_LC[i] = 0;
		average_flow_RC[i] = 0;
	}
}

void obstacle_avoider_periodic() {

	// Initialize variables
	FEATURELESS      = FALSE;
	FRONTAL_OBSTACLE = FALSE;
	SIDE_OBSTACLE    = FALSE;

	average_flow_LC[2] = regions[1].average;
	average_flow_RC[2] = regions[2].average;

	for (int j = 0; j <4; ++j) {
		featureless_indicator[j] = 0;
	}

	if (TURNING ==0) {

		for (int i = 0; i < 4; ++i) {

			// Threshold is dependent on region location
			float t = (i > 0 && i < 3) ? threshold_feature_close : threshold_feature_far;

			// If counter is below threshold, set featureless
			if (regions[i].counter < t) {
				featureless_indicator[i] = 1;
				FEATURELESS = 1;
			}
		}

		if (FEATURELESS == 0) {

			// If flow is above threshold, increase the counters
			counter_flow_LC = 0;
			counter_flow_RC = 0;

			for (int i = 0; i <(MEMORY_FRONTAL); ++i) {
				if (average_flow_LC[i] > frontal_threshold) {
					counter_flow_LC++;
				}
				if (average_flow_RC[i] > frontal_threshold) {
					counter_flow_RC++;
				}
			}

			// If counters are above threshold, set frontal obstacle
			if (counter_flow_LC > (MEMORY_FRONTAL-1) && counter_flow_RC > (MEMORY_FRONTAL-1)) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount_Frontal = changeHeading_OF_Frontal;

			} else if (counter_flow_LC > (MEMORY_FRONTAL-1)) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount_Frontal = changeHeading_OF_Lateral;

			} else if (counter_flow_RC > (MEMORY_FRONTAL-1)) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount_Frontal = -changeHeading_OF_Lateral;
			}

//			if (FRONTAL_OBSTACLE == 0) {
//
//				 // If the difference in flow is above threshold, set side obstacle
//				 if (abs(regions[3].average - regions[0].average) > sideFar_threshold) {
//					SIDE_OBSTACLE = 1;
//
//					if (regions[3].average > regions[0].average) {
//						changeHeading_amount_Side = -changeHeading_OF_sideFar;
//					} else {
//						changeHeading_amount_Side = changeHeading_OF_sideFar;
//					}
//				}
//			}
		}
	}

	// Prepare variables for the next iteration
	for (int i = 0; i <(MEMORY_FRONTAL-1) ; ++i) {
		average_flow_LC[i] = average_flow_LC[i+1];
		average_flow_RC[i] = average_flow_RC[i+1];
	}

	DOWNLINK_SEND_OBJECT_DETECTION(DefaultChannel, DefaultDevice,
								   &TURNING,
								   &FEATURELESS,
								   &FRONTAL_OBSTACLE,
								   &SIDE_OBSTACLE,
								   &regions[0].average,
								   &regions[1].average,
								   &regions[2].average,
								   &regions[3].average,
								   &regions[0].counter,
								   &regions[1].counter,
								   &regions[2].counter,
								   &regions[3].counter);

}

//////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS USED IN THE FLIGHT PLAN

uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	struct EnuCoor_i new_coor;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	// Now determine where to place the waypoint you want to go to
	new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	new_coor.z = pos->z; // Keep the height the same

	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	return FALSE;
}

uint8_t moveWaypointAngle(uint8_t waypoint, float distanceMeters, float angle){
	struct EnuCoor_i new_coor;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	float pi = 22/7;

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading) + angle*pi/180);
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading) + angle*pi/180);

	// Now determine where to place the waypoint you want to go to
	new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	new_coor.z = pos->z; // Keep the height the same

	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	return FALSE;
}

uint8_t changeHeading_Featureless() {

	// if no features close left && close right -> rotate 180???
	if (featureless_indicator[1] && featureless_indicator[2]) {
		changeHeading_amount_Featureless = 180;
	}

	// if no features close left -> big rotate to right
	else if (featureless_indicator[1]) {
		changeHeading_amount_Featureless = changeHeading_Featureless_Close;
	}

	// if no features close right -> big rotate to left
	else if (featureless_indicator[2]) {
		changeHeading_amount_Featureless = -changeHeading_Featureless_Close;
	}

	// if no features far left -> small rotate to right
	else if (featureless_indicator[0]) {
		changeHeading_amount_Featureless = changeHeading_Featureless_Far;
	}

	// if no features far right -> small rotate to left
	else if (featureless_indicator[3]) {
		changeHeading_amount_Featureless = -changeHeading_Featureless_Far;
	}

	return FALSE;
}

uint8_t changeHeading_Outside(){

	int r = rand() % 20;
	changeHeading_amount_Outside = 80 + r;
	return FALSE;
}