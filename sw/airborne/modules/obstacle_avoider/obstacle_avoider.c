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

#define MEMORY 6

int32_t incrementForAvoidance;

float vector_diver[MEMORY];
struct divergence_t diver;

float threshold_diver = 0.1;
float max_diver = 0.6;
int threshold_obstacle = 4;

int counter_diver;

// Object detection
uint8_t Object = FALSE;


void obstacle_avoider_init() {

	// Seed the random number generator with current time
	srand(time(NULL));
	chooseRandomIncrementAvoidance();

	// Initialize the variables related with obstacle detection.
	diver.OLD_5 = 0;
	diver.OLD_4 = 0;
	diver.OLD_3 = 0;
	diver.OLD_2 = 0;
	diver.OLD_1 = 0;
	diver.NOW   = 0;

}

void obstacle_avoider_periodic() {

	// Update current value of divergence
	diver.NOW   = divergence;

	// Update the vector
	vector_diver[0] = diver.OLD_5;
	vector_diver[1] = diver.OLD_4;
	vector_diver[2] = diver.OLD_3;
	vector_diver[3] = diver.OLD_2;
	vector_diver[4] = diver.OLD_1;
	vector_diver[5] = diver.NOW;

	// Check the number of positives above the threshold
	counter_diver = 0;
	for (int i = 0; i <MEMORY ; ++i) {
		if (vector_diver[i] > threshold_diver && vector_diver[i] < max_diver) {
			counter_diver++;
		}
	}

	// Check if we are facing an obstacle
	Object = FALSE;
	if (counter_diver >= threshold_obstacle) {
		Object = TRUE;
	}

   // Prepare variables for the next iteration
	diver.OLD_5 = diver.OLD_4;
	diver.OLD_4 = diver.OLD_3;
	diver.OLD_3 = diver.OLD_2;
	diver.OLD_2 = diver.OLD_1;
	diver.OLD_1 = diver.NOW;

	DOWNLINK_SEND_OBJECT_DETECTION(DefaultChannel, DefaultDevice, &counter_diver);

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