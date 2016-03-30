/*
 * Copyright (C) Group 05
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/obstacle_avoider.h"
 * @author Group 05
 */

#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H
#include <inttypes.h>

extern float threshold_feature_far;
extern float threshold_feature_close;

extern void obstacle_avoider_init(void);
extern void obstacle_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointAngle(uint8_t waypoint, float distanceMeters, float angle);
extern uint8_t changeHeading_Featureless(void);
extern uint8_t changeHeading_Outside(void);

// Featureless region
extern int featureless_indicator[4];

// Change heading
extern float changeHeading_amount_Outside;
extern float changeHeading_amount_Featureless;
extern float changeHeading_amount_Frontal;
extern float changeHeading_amount_Side;

// Exceptions
extern uint8_t FEATURELESS;
extern uint8_t FRONTAL_OBSTACLE;
extern uint8_t SIDE_OBSTACLE;

#endif

