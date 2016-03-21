/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H
#include <inttypes.h>

extern int32_t incrementForAvoidance;
extern void obstacle_avoider_init(void);
extern void obstacle_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);

// Divergence condition
struct divergence_t {
    float OLD_5;
    float OLD_4;
    float OLD_3;
    float OLD_2;
    float OLD_1;
    float NOW;
};

// Divergence condition
struct exceptions_t {
    float OLD_2;
    float OLD_1;
    float NOW;
};

// Object detection
extern uint8_t Object;

#endif

