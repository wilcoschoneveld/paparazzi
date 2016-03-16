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

uint8_t safeToGoForwards=FALSE;
int tresholdColorCount = 200;
int32_t incrementForAvoidance;

void obstacle_avoider_init() {
	// Initialise random values
	srand(time(NULL));
	chooseRandomIncrementAvoidance();
}
void obstacle_avoider_periodic() {

	//safeToGoForwards = color_count < tresholdColorCount;

}

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

