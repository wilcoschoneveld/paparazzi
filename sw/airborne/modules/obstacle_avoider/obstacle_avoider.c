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

#define MEMORY_DIVERGENCE 6
#define MEMORY_EXCEPTIONS 3

int32_t incrementForAvoidance;

// Initialize structures
struct divergence_t divergence_b;
struct exceptions_t turning;
struct exceptions_t featureless;
struct exceptions_t featureless_right;
struct exceptions_t featureless_left;

// Initialize vectors
float vector_divergence_b[MEMORY_DIVERGENCE];
float vector_turning[MEMORY_EXCEPTIONS];
float vector_featureless[MEMORY_EXCEPTIONS];
float vector_featureless_right[MEMORY_EXCEPTIONS];
float vector_featureless_left[MEMORY_EXCEPTIONS];

// Counter
int counter;

// Check if the aircraft is turning
float threshold_turning = 0.2;
uint8_t TURNING = FALSE;

// Featureless object in the front
int threshold_featureless = 5;
uint8_t FEATURELESS = FALSE;

// Featureless object in the right
int threshold_featureless_sides = 3;
uint8_t FEATURELESS_RIGHT = FALSE;
uint8_t FEATURELESS_LEFT  = FALSE;

// Turn
uint8_t TURN_RIGHT = FALSE;
uint8_t TURN_LEFT  = FALSE;
int flag_right = 0;
int flag_left  = 0;

// Check whether there is an obstacle
int counter_positive;
int counter_negative;
float min_threshold = 1;
float max_threshold = 5;
int obstacle_threshold = 3;

void obstacle_avoider_init() {

	// Seed the random number generator with current time
	srand(time(NULL));
	chooseRandomIncrementAvoidance();

	// Initialize the variables related with the balance of lateral optical flow.
	divergence_b.OLD_5 = 0;
	divergence_b.OLD_4 = 0;
	divergence_b.OLD_3 = 0;
	divergence_b.OLD_2 = 0;
	divergence_b.OLD_1 = 0;
	divergence_b.NOW   = 0;

	// Initialize the variables related with turning
	turning.OLD_2 = 0;
	turning.OLD_1 = 0;
	turning.NOW   = 0;

	// Initialize the variables related with featureless objects
	featureless.OLD_2 = 0;
	featureless.OLD_1 = 0;
	featureless.NOW   = 0;

	featureless_right.OLD_2 = 0;
	featureless_right.OLD_1 = 0;
	featureless_right.NOW   = 0;

	featureless_left.OLD_2 = 0;
	featureless_left.OLD_1 = 0;
	featureless_left.NOW   = 0;

}

void obstacle_avoider_periodic() {

	// Update current values
	divergence_b.NOW      = divergence;
	turning.NOW           = yaw_rate;
	featureless.NOW       = feature_cnt;
	featureless_right.NOW = counter_right;
	featureless_left.NOW  = counter_left;

	// Update vectors
	vector_divergence_b[0] = divergence_b.OLD_5;
	vector_divergence_b[1] = divergence_b.OLD_4;
	vector_divergence_b[2] = divergence_b.OLD_3;
	vector_divergence_b[3] = divergence_b.OLD_2;
	vector_divergence_b[4] = divergence_b.OLD_1;
	vector_divergence_b[5] = divergence_b.NOW;

	vector_turning[0] = turning.OLD_2;
	vector_turning[1] = turning.OLD_1;
	vector_turning[2] = turning.NOW;

	vector_featureless[0] = featureless.OLD_2;
	vector_featureless[1] = featureless.OLD_1;
	vector_featureless[2] = featureless.NOW;

	vector_featureless_right[0] = featureless_right.OLD_2;
	vector_featureless_right[1] = featureless_right.OLD_1;
	vector_featureless_right[2] = featureless_right.NOW;

	vector_featureless_left[0] = featureless_left.OLD_2;
	vector_featureless_left[1] = featureless_left.OLD_1;
	vector_featureless_left[2] = featureless_left.NOW;

	// Check turning
	counter = 0;
	for (int i = 0; i <MEMORY_EXCEPTIONS ; ++i) {
		if (abs(vector_turning[i]) > threshold_turning) {
			counter++;
		}
	}

	TURNING = FALSE;
	if (counter > 0) {
		TURNING = TRUE;
	}

	// Check frontal featureless
	counter = 0;
	for (int i = 0; i <MEMORY_EXCEPTIONS ; ++i) {
		if (vector_featureless[i] < threshold_featureless) {
			counter++;
		}
	}

	FEATURELESS = FALSE;
	if (counter == MEMORY_EXCEPTIONS) {
		FEATURELESS = TRUE;
	}

	// Check lateral featureless
	counter = 0;
	for (int i = 0; i <MEMORY_EXCEPTIONS ; ++i) {
		if (vector_featureless_right[i] < threshold_featureless_sides) {
			counter++;
		}
	}

	FEATURELESS_RIGHT = FALSE;
	if (counter == MEMORY_EXCEPTIONS) {
		FEATURELESS_RIGHT = TRUE;
	}

	counter = 0;
	for (int i = 0; i <MEMORY_EXCEPTIONS ; ++i) {
		if (vector_featureless_left[i] < threshold_featureless_sides) {
			counter++;
		}
	}

	FEATURELESS_LEFT = FALSE;
	if (counter == MEMORY_EXCEPTIONS) {
		FEATURELESS_LEFT = TRUE;
	}

	// Turn right or turn left
	TURN_RIGHT = FALSE;
	TURN_LEFT  = FALSE;
	flag_right = 0;
	flag_left  = 0;
	if (FEATURELESS_LEFT && !FEATURELESS_RIGHT) {
		TURN_RIGHT = TRUE;
		flag_right = 1;
	} else if (!FEATURELESS_LEFT && FEATURELESS_RIGHT) {
		TURN_LEFT = TRUE;
		flag_left  = 1;
	}

	// If none of the previous conditions is met, then check if there's a lateral obstacle
	if (!TURNING && !FEATURELESS && !TURN_RIGHT && !TURN_LEFT) {
		counter_positive = 0;
		counter_negative = 0;
		for (int i = 0; i <MEMORY_DIVERGENCE ; ++i) {
			if (vector_divergence_b[i] > min_threshold && vector_divergence_b[i] < max_threshold) {
				counter_positive++;
			} else if (vector_divergence_b[i] < -min_threshold && vector_divergence_b[i] > -max_threshold) {
				counter_negative++;
			}
		}
		if (counter_positive >= obstacle_threshold) {
			TURN_LEFT = TRUE;
			flag_left  = 1;
		} else if (counter_negative >= obstacle_threshold) {
			TURN_RIGHT = TRUE;
			flag_right = 1;
		}
	}

   // Prepare variables for the next iteration
	divergence_b.OLD_5 = divergence_b.OLD_4;
	divergence_b.OLD_4 = divergence_b.OLD_3;
	divergence_b.OLD_3 = divergence_b.OLD_2;
	divergence_b.OLD_2 = divergence_b.OLD_1;
	divergence_b.OLD_1 = divergence_b.NOW;

	turning.OLD_2 = turning.OLD_1;
	turning.OLD_1 = turning.NOW;

	featureless.OLD_2 = featureless.OLD_1;
	featureless.OLD_1 = featureless.NOW;

	featureless_right.OLD_2 = featureless_right.OLD_1;
	featureless_right.OLD_1 = featureless_right.NOW;

	featureless_left.OLD_2 = featureless_left.OLD_1;
	featureless_left.OLD_1 = featureless_left.NOW;

	DOWNLINK_SEND_OBJECT_DETECTION(DefaultChannel, DefaultDevice, &TURNING, &FEATURELESS, &FEATURELESS_RIGHT, &FEATURELESS_LEFT, &TURN_RIGHT, &TURN_LEFT, &flag_right, &flag_left);

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