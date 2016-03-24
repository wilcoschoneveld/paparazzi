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

#define MEMORY 3

int32_t incrementForAvoidance;
int randomIncrement;

// Initialize vectors
float turning[MEMORY];
float counter_RF[MEMORY];
float counter_RC[MEMORY];
float counter_LF[MEMORY];
float counter_LC[MEMORY];

// Counter
int counter_turning;

// Change heading
float changeHeading_amount = 0;

// FIRST CONDITION: Check if the aircraft is turning
float threshold_turning = 0.2;

uint8_t TURNING = FALSE;

// SECOND CONDITION: Check featureless regions
float counter_average[4];

int threshold_feature_far = 3;
int threshold_feature_close = 5;

uint8_t FEATURELESS = FALSE;
int featureless_indicator[4];

float changeHeading_Featureless_Far   = 45;
float changeHeading_Featureless_Close = 90;

// THIRD CONDITION: Check frontal obstacle

float frontal_threshold_min = 10;
float frontal_threshold_max = 30;
float changeHeading_OF_Frontal = 180;
float changeHeading_OF_Lateral = 90;
uint8_t FRONTAL_OBSTACLE = FALSE;

// FOURTH CONDITION: Check side obstacles
float sideClose_threshold = 10;
float sideFar_threshold   = 10;
float changeHeading_OF_sideClose = 60;
float changeHeading_OF_sideFar   = 30;
uint8_t SIDE_OBSTACLE = FALSE;

void obstacle_avoider_init() {

	// Seed the random number generator with current time
	srand(time(NULL));
	chooseRandomIncrementAvoidance();

	// Initialize the variables related with turning
	turning[0] = 0;
	turning[1] = 0;

	// Initialize the variables related with feature detection
	counter_RF[0] = 0;
	counter_RF[1] = 0;

	counter_RC[0] = 0;
	counter_RC[1] = 0;

	counter_LF[0] = 0;
	counter_LF[1] = 0;

	counter_LC[0] = 0;
	counter_LC[1] = 0;
}

void obstacle_avoider_periodic() {

	// Initialize variables
	TURNING = FALSE;
	FEATURELESS = FALSE;
	FRONTAL_OBSTACLE = FALSE;
	SIDE_OBSTACLE = FALSE;
	featureless_indicator[0] = 0;
	featureless_indicator[1] = 0;
	featureless_indicator[2] = 0;
	featureless_indicator[3] = 0;

	// Update current values
	turning[2]    = yaw_rate;
	counter_RF[2] = regions[3].counter;
	counter_RC[2] = regions[2].counter;
	counter_LC[2] = regions[1].counter;
	counter_LF[2] = regions[0].counter;

	// Check turning
	counter_turning = 0;
	for (int i = 0; i <MEMORY ; ++i) {
		if (abs(turning[i]) > threshold_turning) {
			counter_turning++;
		}
	}

	if (counter_turning > 0) {
		TURNING = 1;
	} else {

		// Check featureless regions
		counter_average[0] = (counter_LF[2] + counter_LF[1] + counter_LF[0]) / 3;
		counter_average[1] = (counter_LC[2] + counter_LC[1] + counter_LC[0]) / 3;
		counter_average[2] = (counter_RC[2] + counter_RC[1] + counter_RC[0]) / 3;
		counter_average[3] = (counter_RF[2] + counter_RF[1] + counter_RF[0]) / 3;

		if (counter_average[0] < threshold_feature_far) {
			featureless_indicator[0] = 1;
			FEATURELESS = 1;
		} else {
			featureless_indicator[0] = 0;
		}

		if (counter_average[1] < threshold_feature_close) {
			featureless_indicator[1] = 1;
			FEATURELESS = 1;
		} else {
			featureless_indicator[1] = 0;
		}

		if (counter_average[2] < threshold_feature_close) {
			featureless_indicator[2] = 1;
			FEATURELESS = 1;
		} else {
			featureless_indicator[2] = 0;
		}

		if (counter_average[3] < threshold_feature_far) {
			featureless_indicator[3] = 1;
			FEATURELESS = 1;
		} else {
			featureless_indicator[3] = 0;
		}

		if (FEATURELESS ==0) {

			if (regions[1].average > frontal_threshold_min && regions[1].average < frontal_threshold_max && regions[2].average > frontal_threshold_min && regions[2].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = changeHeading_OF_Frontal;
			} else if (regions[1].average > frontal_threshold_min && regions[1].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = changeHeading_OF_Lateral;
			} else if (regions[2].average > frontal_threshold_min && regions[2].average < frontal_threshold_max) {
				FRONTAL_OBSTACLE = 1;
				changeHeading_amount = -changeHeading_OF_Lateral;
			}

			if (FRONTAL_OBSTACLE  ==0) {
				// Check side obstacles

				if (abs(regions[2].average - regions[1].average) > sideClose_threshold) {
					SIDE_OBSTACLE = 1;
					if (regions[2].average > regions[1].average) {
						changeHeading_amount = -changeHeading_OF_sideClose;
					} else {
						changeHeading_amount = changeHeading_OF_sideClose;
					}

				} else if (abs(regions[3].average - regions[0].average) > sideFar_threshold) {
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

	// Prepare variables for the next iteration
	turning[0] = turning[1];
	turning[1] = turning[2];

	counter_RF[0] = counter_RF[1];
	counter_RF[1] = counter_RF[2];

	counter_RC[0] = counter_RC[1];
	counter_RC[1] = counter_RC[2];

	counter_LF[0] = counter_LF[1];
	counter_LF[1] = counter_LF[2];

	counter_LC[0] = counter_LC[1];
	counter_LC[1] = counter_LC[2];

	DOWNLINK_SEND_OBJECT_DETECTION(DefaultChannel, DefaultDevice, &TURNING, &FEATURELESS, &FRONTAL_OBSTACLE, &SIDE_OBSTACLE, &regions[0].average, &regions[1].average, &regions[2].average, &regions[3].average, &counter_average[0], &counter_average[1], &counter_average[2], &counter_average[3]);

}

//////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS USED IN THE FLIGHT PLAN

/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */

uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
	*heading = *heading + increment;
	// Check if your turn made it go out of bounds...
	INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
	return FALSE;
}

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

uint8_t chooseRandomIncrementAvoidance(){

	int r = rand() % 2;
	if(r==0){
		incrementForAvoidance=350;
	}
	else{
		incrementForAvoidance=-350;
	}
	return FALSE;
}

uint8_t changeHeading_Featureless(){

	if (featureless_indicator[0] == 1 && featureless_indicator[1] == 0 && featureless_indicator[2] == 0 && featureless_indicator[3] == 0) {
		changeHeading_amount = changeHeading_Featureless_Far;
	} else if ((featureless_indicator[1] == 1 && featureless_indicator[2] == 0 && featureless_indicator[3] == 0) || (featureless_indicator[0] == 1 && featureless_indicator[1] == 0 && featureless_indicator[2] == 1 && featureless_indicator[3] == 0)) {
		changeHeading_amount = changeHeading_Featureless_Close;
	} else if ((featureless_indicator[0] == 0 && featureless_indicator[1] == 0 && featureless_indicator[2] == 1) || (featureless_indicator[0] == 0 && featureless_indicator[1] == 1 && featureless_indicator[2] == 0 && featureless_indicator[3] == 1)) {
		changeHeading_amount = -changeHeading_Featureless_Close;
	} else if (featureless_indicator[0] == 0 && featureless_indicator[1] == 0 && featureless_indicator[2] == 0 && featureless_indicator[3] == 1) {
		changeHeading_amount = -changeHeading_Featureless_Far;
	} else if (featureless_indicator[0] == 1 && featureless_indicator[1] == 0 && featureless_indicator[2] == 0 && featureless_indicator[3] == 1) {
		changeHeading_amount = 0;
	} else if (featureless_indicator[1] == 0 && featureless_indicator[2] == 1) {
		changeHeading_amount = 180;
	}

	return FALSE;
}

uint8_t changeHeading_Outside(){

	int r = rand() % 40;
	changeHeading_amount = 150 + r;
	return FALSE;
}