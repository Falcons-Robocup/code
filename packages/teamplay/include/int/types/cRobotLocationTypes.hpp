 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
