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

extern int32_t incrementForAvoidance;
extern int randomIncrement;
extern void obstacle_avoider_init(void);
extern void obstacle_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointAngle(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern uint8_t changeHeading_Featureless(void);
extern uint8_t changeHeading_Outside(void);

// Divergence condition
struct exceptions_t {
    float OLD_2;
    float OLD_1;
    float NOW;
};

// Featureless region
extern int featureless_indicator[4];
extern float flow_average[4];
extern float changeHeading_amount;

// Exceptions
extern uint8_t FEATURELESS;
extern uint8_t FRONTAL_OBSTACLE;
extern uint8_t SIDE_OBSTACLE;

#endif

