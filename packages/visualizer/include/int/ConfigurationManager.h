// Copyright 2016-2020 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigurationManager.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

#include "falconsCommon.hpp"

// Constants for now, to be moved to an interface with customizable implementation later.

const int _NR_OF_ROBOTS_PER_TEAM = MAX_ROBOTS;

const double _LINE_THICKNESS = 7.5;
const double _BALL_DIAMETER = 0.3;
const double _CORNER_CIRCLE_RADIUS = 0.75;
const double _BLACK_POINT_WIDTH = 0.0;
const double _BLACK_POINT_LENGTH = 0.0;

const double _OBSTACLE_DIAMETER = 0.5; // Fixed diameter assumed for now
const double _OBSTACLE_OPACITY = 0.3;

#endif // CONFIGURATION_MANAGER_H
