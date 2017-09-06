 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cObstacleFacilities.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/facilities/cObstacleFacilities.hpp"
#include <boost/thread/thread.hpp>

void cObstacleFacilities::projectObstaclesOnForbiddenAreaSide(Vector2D& startPoint, Vector2D& endPoint, std::vector<pp_obstacle_struct_t>& obstacles)
{
    //Starting point
    Vector2D deltaVec = endPoint - startPoint;

    #define ROBOT_RADIUS 75.0 / 100.0 / 2.0 // 75cm diameter = 37.5cm radius = 0.375m

    Vector2D currentProjectionDistance;
    double distBetweenProjectedObstacles = (ROBOT_RADIUS);
    for (size_t j = 1; (currentProjectionDistance.size() + distBetweenProjectedObstacles) < deltaVec.size(); j++)
    {
        //TRACE("j=%d ; currProjDist=%12.9f ; projDist=%12.9f", j, currentProjectionDistance.size(), projectionDistance.size());
        currentProjectionDistance = deltaVec.normalized() * ( j * distBetweenProjectedObstacles );
        Vector2D currentProjectedObstPos = startPoint + currentProjectionDistance;

        pp_obstacle_struct_t currentProjectedObst;
        currentProjectedObst.location.x = currentProjectedObstPos.x;
        currentProjectedObst.location.y = currentProjectedObstPos.y;
        currentProjectedObst.location.phi = 0.0;
        currentProjectedObst.velocity.x = 0.0;
        currentProjectedObst.velocity.y = 0.0;
        currentProjectedObst.velocity.phi = 0.0;

        obstacles.push_back(currentProjectedObst);
    }
}

void cObstacleFacilities::projectObstaclesOnForbiddenArea(polygon2D& area, std::vector<pp_obstacle_struct_t>& obstacles)
{
    std::vector<linepoint2D> linepoints = area.getLinepoints();
	for(auto it = linepoints.begin(); it != linepoints.end(); it++)
	{
		Vector2D src = it->getSourceVector2D();
		Vector2D dst = it->getDestinationVector2D();
		projectObstaclesOnForbiddenAreaSide(src, dst, obstacles);
	}
}
