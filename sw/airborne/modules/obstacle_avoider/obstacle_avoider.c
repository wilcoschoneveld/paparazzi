/*
 * Copyright (C) Mavlabcourse 2016 - Group 05
 *
 * This file is part of paparazzi
 *
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

int randomIncrement;

// Change heading
float changeHeading_amount = 0;

// SECOND CONDITION: Check featureless regions
int featureless_indicator[4];

uint8_t FEATURELESS = FALSE;

int threshold_feature_far   = 3;
int threshold_feature_close = 5;

float changeHeading_Featureless_Far   = 20;
float changeHeading_Featureless_Close = 90;

// THIRD CONDITION: Check frontal obstacle
uint8_t FRONTAL_OBSTACLE = FALSE;

float frontal_threshold_min = 10;
float frontal_threshold_max = 50;

float changeHeading_OF_Frontal = 180;
float changeHeading_OF_Lateral = 90;

// FOURTH CONDITION: Check side obstacles
uint8_t SIDE_OBSTACLE = FALSE;

float sideFar_threshold   = 10;

float changeHeading_OF_sideFar   = 15;

void obstacle_avoider_periodic() {

	// Initialize variables
	FEATURELESS      = FALSE;
	FRONTAL_OBSTACLE = FALSE;
	SIDE_OBSTACLE    = FALSE;

	for (int j = 0; j <4 ; ++j) {
		featureless_indicator[j] = 0;
	}

	if (TURNING ==0) {

		// Loop through the regions
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

			if (regions[1].average > frontal_threshold_min && regions[1].average < frontal_threshold_max &&
				regions[2].average > frontal_threshold_min && regions[2].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = changeHeading_OF_Frontal;
			} else if (regions[1].average > frontal_threshold_min && regions[1].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = changeHeading_OF_Lateral;
			} else if (regions[2].average > frontal_threshold_min && regions[2].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = -changeHeading_OF_Lateral;
			}

			if (FRONTAL_OBSTACLE == 0) {

				 if (abs(regions[3].average - regions[0].average) > sideFar_threshold) {
					SIDE_OBSTACLE = 1;
					if (regions[3].average > regions[0].average) {
						changeHeading_amount = -changeHeading_OF_sideFar;
					} else {
						changeHeading_amount = changeHeading_OF_sideFar;
					}
				}
			}

		}
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

uint8_t moveWaypointAngle(uint8_t waypoint, float distanceMeters){
	struct EnuCoor_i new_coor;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading) + changeHeading_amount);
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading) + changeHeading_amount);

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
		changeHeading_amount = 180;
	}

	// if no features close left -> big rotate to right
	else if (featureless_indicator[1]) {
		changeHeading_amount = changeHeading_Featureless_Close;
	}

	// if no features close right -> big rotate to left
	else if (featureless_indicator[2]) {
		changeHeading_amount = -changeHeading_Featureless_Close;
	}

	// if no features far left -> small rotate to right
	else if (featureless_indicator[0]) {
		changeHeading_amount = changeHeading_Featureless_Far;
	}

	// if no features far right -> small rotate to left
	else if (featureless_indicator[3]) {
		changeHeading_amount = -changeHeading_Featureless_Far;
	}

	return FALSE;
}

uint8_t changeHeading_Outside(){

	int r = rand() % 20;
	changeHeading_amount = 80 + r;
	return FALSE;
}