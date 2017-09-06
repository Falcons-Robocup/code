 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ConfigurationManager.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

// Constants for now, to be moved to an interface with customizable implementation later.

const int _NR_OF_ROBOTS_PER_TEAM = 6;

const double _FIELD_LENGTH = 18.0;
const double _FIELD_WIDTH = 12.0;
const double _LINE_THICKNESS = 0.13;
const double _GOAL_AREA_LENGTH = 0.75;
const double _GOAL_AREA_WIDTH = 3.5;
const double _PENALTY_AREA_LENGTH = 2.25;
const double _PENALTY_AREA_WIDTH = 6.5;
const double _CENTER_CIRCLE_RADIUS = 2.0;
const double _BALL_DIAMETER = 0.3;
const double _CORNER_CIRCLE_RADIUS = 0.75;
const double _PENALTY_MARK_DISTANCE = 3.0;
const double _BLACK_POINT_WIDTH = 0.0;
const double _BLACK_POINT_LENGTH = 0.0;
const double _ROBOT_RADIUS = 0.5;

const double _OBSTACLE_DIAMETER = 0.3; // Fixed diameter assumed for now

#endif // CONFIGURATION_MANAGER_H
