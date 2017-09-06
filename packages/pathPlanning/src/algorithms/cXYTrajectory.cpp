 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTrajectoryPathPlanning.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: Erik Kouters
 */

#include <boost/algorithm/string.hpp>
#include <sstream>
#include <fstream>

#include "int/algorithms/cXYTrajectory.hpp"

#include "cDiagnosticsEvents.hpp"
#include <FalconsCommon.h>

#define G_GROUP_GAP_DISTANCE 2.0 * ROBOT_RADIUS
#define SUBTARGET_DIST 2.0 * ROBOT_RADIUS
#define SUBTARGET_EXTENSION_FACTOR 0.0 //Subtarget = subtarget + (subtarget - currPos).normalize() * SUBTARGET_EXTENSION_FACTOR

// Used to compare obstacles for the std::set
struct posval_compare {
    bool operator() (const pp_obstacle_struct_t& lhs, const pp_obstacle_struct_t& rhs) const{
        if (lhs.location.x < rhs.location.x)
            return true;
        else if (lhs.location.y < rhs.location.y)
            return true;
        else
            return false;
    }
};

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYTrajectory::execute()
{
    TRACE("> target: %s", _ppData.pos.tostr());
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    TRACE("PP_FREEZE got position");

    // Get obstacles from cPathPlanningData
    std::vector<pp_obstacle_struct_t> obstacles;
    _main->_ppData->getOnlyObstacles(obstacles);

    TRACE("PP_FREEZE got all obstacles");

    // Get forbiddenAreas from cPathPlanningData
    std::vector<pp_obstacle_struct_t> forbiddenAreas;
    _main->_ppData->getForbiddenAreas(forbiddenAreas);

    TRACE("PP_FREEZE got forbidden areas");

    // Get ballPos from cPathPlanningData
    Position2D ballPos;
    _main->_ppData->getBallPos(ballPos);

    bool ballValid;
    _main->_ppData->getBallValid(ballValid);

    // CONTAINMENT NAGOYA
    // Remove all obstacles within 1m of the ball. (Exclude the forbidden areas!)
    if (ballValid)
    {
        std::vector<pp_obstacle_struct_t>::iterator it;
        for (it = obstacles.begin(); it != obstacles.end(); /* do not increase iterator */ )
        {
            if (calc_distance(ballPos, it->location) < 1.0)
            {
                it = obstacles.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    // Merge forbiddenAreas into obstacles
    obstacles.insert( obstacles.end(), forbiddenAreas.begin(), forbiddenAreas.end() );

    TRACE("PP_FREEZE Starting algorithm with nr_obs: %d", obstacles.size());

    Position2D subTarget = _ppData.pos;

    // Guard with moving at least 10cm
    if ((currPos.xy() - _ppData.pos.xy()).size() > 0.1)
    {

        // 1. Determine B = Set of all obstacles in the direct path from robot position to target position, taking also the diameter of the obstacles and the robot into account.
        std::vector<pp_obstacle_struct_t> B;
        for ( int i = 0; i < (int)obstacles.size(); i++)
        {
            // Pre-compute a_i and b_i for all obstacles, and store in the struct for later reuse.
            Vector2D obstVec = Vector2D(obstacles[i].location.x, obstacles[i].location.y);
            obstacles[i].a_i = ((_ppData.pos.xy() - currPos.xy()) * (obstVec - currPos.xy())) / (_ppData.pos.xy() - currPos.xy()).size();
            obstacles[i].b_i = ((_ppData.pos.xy() - currPos.xy()).CrossProduct((obstVec - currPos.xy()))) / (_ppData.pos.xy() - currPos.xy()).size();

            // Equation (4) from the paper.
            if ( ( 0 < obstacles[i].a_i ) &&
                 ( obstacles[i].a_i < (_ppData.pos.xy() - currPos.xy()).size() ) &&
                 ( fabs(obstacles[i].b_i) < (ROBOT_RADIUS*2.0) )) // r_r + r_i : robot radius + obstacle radius
            {
                B.push_back(obstacles[i]);
                //TRACE("DIST FROM LINE: %6.2f", b_i);
                //TRACE("OBST IN PATH: X: %6.2f, Y: %6.2f", obstVec.x, obstVec.y);
            }

        }

        TRACE("PP_FREEZE Computed step 1.");

        // 2. Determine f = The obstacle from B that is the nearest to the robot
        double minDist = 100.0;
        int B_idx = -1;
        for ( int i = 0; i < (int)B.size(); i++)
        {
            // Find obstacle with the smallest a_i.
            // Equation (5) from the paper.
            if (B[i].a_i < minDist)
            {
                minDist = B[i].a_i;
                B_idx = i;
            }
        }

        TRACE("PP_FREEZE Computed step 2.");

        if (B_idx != -1)
        {
            //TRACE("NEAREST OBST: X: %6.2f, Y: %6.2f", B[B_idx].location.x, B[B_idx].location.y);

            // 3. Determine G = The set of transitive obstacles that are within the range of 2r_r of each other, starting from f.
            //    "In our approach, the group G will be avoided instead of object f only, otherwise the robot could get stuck in the group G."
            //    G = {f};
            //    while True:
            //       Gsize = len(G)
            //       for g in G:
            //          for o in obstacles:
            //             if dist(g, o) < 2r_r (=diameter):
            //                G += o
            //       if len(G) == Gsize: # size has not increased
            //          break

            // Use G_set to build a set. When done, add elements from set to G.
            std::set<pp_obstacle_struct_t>::iterator itG;
            std::set<pp_obstacle_struct_t, posval_compare> G_set;
            G_set.insert(B[B_idx]);
            while (true)
            {
                size_t G_size = G_set.size();

                for (itG = G_set.begin(); itG != G_set.end(); ++itG)
                {
                    for (size_t j = 0; j < obstacles.size(); j++)
                    {
                        // Compute distance between obstacle in G and all obstacles.
                        double dist = calc_hypothenusa(fabs(itG->location.x - obstacles[j].location.x), fabs(itG->location.y - obstacles[j].location.y));

                        //TRACE("Dist between (%6.2f,%6.2f) and (%6.2f,%6.2f) is %6.2f", itG->location.x, itG->location.y, obstacles[j].location.x, obstacles[j].location.y, dist);

                        // Distance is from center of obstacles.
                        // First subtract the radius from each obstacle, and see if the robot's diameter fits through the gap.
                        if ((dist - 2.0*ROBOT_RADIUS) < G_GROUP_GAP_DISTANCE)
                        {
                            G_set.insert(obstacles[j]);
                        }
                    }
                }

                if (G_set.size() == G_size)
                {
                    // Size has not increased -- break the while loop.
                    break;
                }
            }

            TRACE("PP_FREEZE Computed step 3. G_set.size() = %d", G_set.size());

            // Convert the set to a vector.
            std::vector<pp_obstacle_struct_t> G;
            for (itG = G_set.begin(); itG != G_set.end(); ++itG)
            {
                G.push_back(*itG);
                //TRACE("OBST IN G: X: %6.2f, Y: %6.2f", itG->location.x, itG->location.y);
            }

            // 4. Compute distance between line (robot -> target) and every object i in G:
            //    "The side to pass the group G is determined by selecting the side with the smallest absolute value of the perpendicular distance b_i for all i in G."
            //    b+_max = 0
            //    b-_max = 0
            //    for i in G:
            //       b_i = dist( line(robot, target), i )
            //       if b_i > 0:
            //          val = abs(b_i + r_r)
            //          if val > b+_max:
            //             b+_max = val
            //       else:
            //          val = abs(b_i - r_r)
            //          if val > b-_max:
            //             b-_max = val
            //    if b+_max >= b-_max:
            //       # left side of G
            //    else:
            //       # right side of G
            double bp_max = 0.0;
            double bm_max = 0.0;
            std::vector<pp_obstacle_struct_t> G_plus;
            std::vector<pp_obstacle_struct_t> G_min;
            for (int i = 0; i < (int)G.size(); i++)
            {
                // Determine if this obstacle belongs in the positive group or negative group.
                if (G[i].b_i > 0)
                {
                    G_plus.push_back(G[i]);

                    // Equation (6a)
                    double val = fabs(G[i].b_i + ROBOT_RADIUS);
                    if (val > bp_max)
                    {
                        bp_max = val;
                    }
                }
                else
                {
                    G_min.push_back(G[i]);

                    // Equation (6b)
                    double val = fabs(G[i].b_i - ROBOT_RADIUS);
                    if (val > bm_max)
                    {
                        bm_max = val;
                    }
                }
            }
            //TRACE("b+_max = %6.2f , b-_max = %6.2f", bp_max, bm_max);

            // Equation (6c)
            std::vector<pp_obstacle_struct_t> G_side_to_use;
            if (bp_max >= bm_max)
            {
                // go for minus side of G
                G_side_to_use = G_min;
                //TRACE("GOING FOR MINUS SIDE OF G");
            }
            else
            {
                // go for plus side of G
                G_side_to_use = G_plus;
                //TRACE("GOING FOR PLUS SIDE OF G");
            }

            // Always consider obstacles on the line robot -> target
            for (size_t i = 0; i < B.size(); i++)
            {
                G_side_to_use.push_back(B[i]);
            }

            TRACE("PP_FREEZE Computed step 4.");

            // 5. Compute for each i \in G the angle alpha. Save the i with highest angle -> alpha_max.
            // Determine "j" below equation (8) -> G_idx
            double alpha_max = 0.0;
            double alpha_j = 0.0;
            int G_idx = -1;

            for (size_t i = 0; i < G_side_to_use.size(); i++)
            {
                // Determine the sign from equation (7)
                double sign = 0.0;
                if ((bm_max - bp_max) < 0)
                    sign = -1.0;
                else if ((bm_max - bp_max) == 0)
                    sign = 0.0;
                else
                    sign = 1.0;

                // Compute the left and right part of equation (7)
                double alpha_left = atan(G_side_to_use[i].b_i / G_side_to_use[i].a_i);
                double distBetweenObstAndRobot = (Vector2D(G_side_to_use[i].location.x, G_side_to_use[i].location.y)-currPos.xy()).size();
                double alpha_right = sign*asin( fmax( fmin( SUBTARGET_DIST/distBetweenObstAndRobot, 0.99), -0.99) ); // asin's argument must be in [-1 < x < 1]
                //TRACE("alpha_left: %6.4f, alpha_right: %6.4f", alpha_left, alpha_right);

                // Equation (7)
                double alpha = alpha_left + alpha_right;
                double abs_alpha = fabs(alpha);

                if (abs_alpha > alpha_max)
                {
                    alpha_max = abs_alpha;
                    alpha_j = alpha;
                    G_idx = i;
                }

                //TRACE("(%6.2f,%6.2f) angle is %6.2f", G_side_to_use[i].location.x, G_side_to_use[i].location.y, alpha);
            }

            TRACE("PP_FREEZE Computed step 5.");

            if (G_idx != -1)
            {
                // Equation (8) -- G_idx == j
                Vector2D rightPart = ((_ppData.pos.xy() - currPos.xy()) / (_ppData.pos.xy() - currPos.xy()).size()) * (Vector2D(G_side_to_use[G_idx].location.x, G_side_to_use[G_idx].location.y) - currPos.xy()).size();
                Vector2D multipliedMatrix = Vector2D( ( (cos(alpha_j)*rightPart.x) - (sin(alpha_j)*rightPart.y) ) , ( (sin(alpha_j)*rightPart.x) + (cos(alpha_j)*rightPart.y) ) );
                Vector2D subtarget = currPos.xy() + multipliedMatrix;

                //TRACE("subtarget pre-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);

                // Put the subtarget further away to prevent the robot from slowing down for the subtarget
                subtarget = subtarget + ((subtarget - currPos.xy()).normalize() * SUBTARGET_EXTENSION_FACTOR);

                //TRACE("subtarget post-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);

                subTarget.x = subtarget.x;
                subTarget.y = subtarget.y;
            }

            TRACE("PP_FREEZE All done.");
        }
    }
    
    TRACE("PP_FREEZE Algorithm done, subX: %6.2f, subY: %6.2f, forbiddenSize %d", subTarget.x, subTarget.y, forbiddenAreas.size());

    // Send obstacles to diagnostics (for the visualizer)
    std::vector<polygon2D> forbAreas;
    std::vector<pp_obstacle_struct_t> onlyObstacles;
    std::vector<linepoint2D> speedVectors;
    _main->_ppData->getForbiddenAreas(forbAreas);
    _main->_ppData->getOnlyObstacles(onlyObstacles);
    _main->_ppData->getProjectedSpeedVectors(speedVectors);

    _main->_ppData->publishObstacles(onlyObstacles, forbAreas, speedVectors);
    
    TRACE("PP_FREEZE Published obstacles");

    /*
     * Double-check whether target setpoint is not in a forbidden area
     */
    bool isCurrentlyInForbiddenArea = false;
    Point2D currPos2D = Point2D(currPos.x, currPos.y);

    for(size_t i = 0; ((i < forbAreas.size()) && (!isCurrentlyInForbiddenArea)); i++)
    {
    	if(forbAreas.at(i).pointExistsInPolygon(currPos2D))
    	{
    		isCurrentlyInForbiddenArea = true;
    		subTarget.x = 0.0;
    		subTarget.y = 0.0;
    		TRACE_INFO("Overwriting subtarget to leave a forbidden area");
    	}
    }

    TRACE("PP_FREEZE checked setpoint in forbidden area");

    // Publish subtarget to diagnostics
    _main->_ppData->publishSubtarget(subTarget.x, subTarget.y);
    //_main->_diagnosticsAdapter.setPathPlanningSubtarget(subTarget.x, subTarget.y);

    _ppData.pos.x = subTarget.x;
    _ppData.pos.y = subTarget.y;
    //return _planningAlgo->updatePathPlanningXY(subTarget);


    /*
    Velocity2D newVel;
    newVel.x = _pidParams.XY_P * (targetPosition.x - currPos.x);
    newVel.y = _pidParams.XY_P * (targetPosition.y - currPos.y);
    newVel.phi = _pidParams.PHI_P * (project_angle_mpi_pi(targetPosition.phi - currPos.phi));
    if(_wmAdapter.isRobotActive())
    {
        _setSpeedAdapter.setSpeed(newVel, currPos);
    }
    */

    /* TODO: call PID controller for driving towards the target */

    /* Limit velocities (is already done in PID) */
    //limitVelocities(targetPosition, currPos, newVel);
    //_setSpeedAdapter.setSpeed(newVel, currPos); 



    TRACE("< subtarget: %s", _ppData.pos.tostr());
}
