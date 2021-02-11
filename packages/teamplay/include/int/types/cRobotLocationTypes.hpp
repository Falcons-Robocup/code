// Copyright 2015-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRobotLocationTypes.hpp
 *
 *  Created on: Dec 4, 2015
 *      Author: Coen Tempelaars
 */

#ifndef CROBOTLOCATIONTYPES_HPP_
#define CROBOTLOCATIONTYPES_HPP_

#include "pose2d.hpp"
#include "velocity2d.hpp"
#include "position2d.hpp"
#include <map>
#include <vector>

typedef uint8_t robotNumber;

typedef struct
{
    geometry::Pose2D position;
    geometry::Velocity2D velocity;
} robotLocation;

typedef std::map<robotNumber, robotLocation> robotLocations;

class robotLocationSorter {
      Point2D _referencePosition;
public:
      robotLocationSorter(Point2D refPos) : _referencePosition(refPos) {}
      bool operator()(const robotLocation o1, const robotLocation o2) const {
    	  	Position2D posO1 = Position2D(o1.position.getX(), o1.position.getY(), 0.0);
    	  	Position2D posO2 = Position2D(o2.position.getX(), o2.position.getY(), 0.0);
    	  	Position2D refPos = Position2D(_referencePosition.x, _referencePosition.y, 0.0);

    	  	return ((refPos - posO1).size() < (refPos - posO2).size());
      }
};

/*! define structure type to quickly be able to sort opponentInfo based on distance
 */
typedef struct
{
	double distance;
	robotLocation theRobotPosition;

} distanceData;

typedef std::vector<distanceData> distanceDataList;


#endif /* CROBOTLOCATIONTYPES_HPP_ */
