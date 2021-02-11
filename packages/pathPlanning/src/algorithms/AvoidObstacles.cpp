// Copyright 2019-2020 jquack (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AvoidObstacles.cpp
 *
 *  Created on: August, 2019
 *      Author: Josefine Quack
 *
 * Algorithm originally implemented by Erik.
 * Paper: http://www.techunited.nl/media/files/realtime_motion_path_generation_using_subtargets_in_a_changing_environment.pdf
 */


#include "int/PathPlanningAlgorithms.hpp"
#include "cDiagnostics.hpp"



// internal constants, types and utilities

typedef struct
{
    vec2d location;
    vec2d velocity;
    float a_i = 0.0;
    float b_i = 0.0;
    float radius = 0.0;
} pp_obstacle_struct_t;

void convertAndAddObstacles(std::vector<pp_obstacle_struct_t> &dst, std::vector<obstacleResult> const &src)
{
    for (auto it = src.begin(); it != src.end(); ++it)
    {
        pp_obstacle_struct_t obst;
        obst.location = it->position;
        obst.velocity = it->velocity;
        dst.push_back(obst);
    }
}

struct posval_compare // for std::set
{
    bool operator() (const pp_obstacle_struct_t& lhs, const pp_obstacle_struct_t& rhs) const
    {
        if (lhs.location.x < rhs.location.x)
            return true;
        else if (lhs.location.x > rhs.location.x)
            return false;
        else if (lhs.location.y < rhs.location.y)
            return true;
        else
            return false;
    }
};


// obstacle avoidance algorithm

void AvoidObstacles::execute(PathPlanningData &data)
{
    // get target, initialize
    Position2D target = data.getSubTarget();
    std::string msg = "target: ";
    msg.append(target.tostr());
    TRACE_FUNCTION(msg.c_str());
    auto config = data.config.obstacleAvoidance;

    // check if functionality is enabled
    if (config.enabled == false)
    {
        TRACE("obstacle avoidance is disabled");
        return;
    }

    // get other data
    Position2D currPos(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);

    // get calculated obstacles, convert to internal type
    std::vector<pp_obstacle_struct_t> obstacles;
    convertAndAddObstacles(obstacles, data.calculatedObstacles); // calculated by CalculateObstacles

    TRACE("Starting algorithm with nr_obs: %d", obstacles.size());

    Position2D subTarget = target;
    bool calculatedSubTarget = false;

    // Guard with moving at least 10cm
    if ((currPos.xy() - target.xy()).size() > 0.1)
    {
        std::vector<pp_obstacle_struct_t> B;
        {
            TRACE_SCOPE("cXYTrajectory#Step1", "");
            // 1. Determine B = Set of all obstacles in the direct path from robot position to target position, taking also the diameter of the obstacles and the robot into account.
            for (int i = 0; i < (int)obstacles.size(); i++)
            {
                // Pre-compute a_i and b_i for all obstacles, and store in the struct for later reuse.
                Vector2D obstVec = Vector2D(obstacles[i].location.x, obstacles[i].location.y);
                obstacles[i].a_i = ((target.xy() - currPos.xy()) * (obstVec - currPos.xy())) / (target.xy() - currPos.xy()).size();
                obstacles[i].b_i = ((target.xy() - currPos.xy()).CrossProduct((obstVec - currPos.xy()))) / (target.xy() - currPos.xy()).size();
                obstacles[i].radius = config.obstacleRadius;
                // ROADMAP: vision/worldModel might know radius better at some point

                // Equation (4) from the paper.
                if (( 0 < obstacles[i].a_i ) &&
                    ( obstacles[i].a_i < (target.xy() - currPos.xy()).size() ) &&
                    ( fabs(obstacles[i].b_i) < (config.robotRadius + obstacles[i].radius) )) // r_r + r_i
                {
                    B.push_back(obstacles[i]);
                    //TRACE("DIST FROM LINE: %6.2f", obstacles[i].b_i);
                    //TRACE("OBST IN PATH: X: %6.2f, Y: %6.2f", obstVec.x, obstVec.y);
                }

            }
        }

        TRACE("Computed step 1.");

        int B_idx = -1;
        {
            TRACE_SCOPE("cXYTrajectory#Step2", "");
            // 2. Determine f = The obstacle from B that is the nearest to the robot
            float minDist = 100.0;

            for (int i = 0; i < (int)B.size(); i++)
            {
                // Find obstacle with the smallest a_i.
                // Equation (5) from the paper.
                if (B[i].a_i < minDist)
                {
                    minDist = B[i].a_i;
                    B_idx = i;
                }
            }
        }

        TRACE("Computed step 2.");

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
            {
                TRACE_SCOPE("cXYTrajectory#Step3", "");
                while (true)
                {
                    size_t G_size = G_set.size();

                    for (itG = G_set.begin(); itG != G_set.end(); ++itG)
                    {
                        for (size_t j = 0; j < obstacles.size(); j++)
                        {
                            // Compute distance between obstacle in G and all obstacles.
                            float dist = calc_hypothenusa(fabs(itG->location.x - obstacles[j].location.x), fabs(itG->location.y - obstacles[j].location.y));

                            //TRACE("Dist between (%6.2f,%6.2f) and (%6.2f,%6.2f) is %6.2f", itG->location.x, itG->location.y, obstacles[j].location.x, obstacles[j].location.y, dist);

                            // Distance is from center of obstacles.
                            // First subtract the radius from each obstacle, and see if the robot's diameter fits through the gap.
                            if ((dist - (config.robotRadius + obstacles[j].radius)) < config.groupGapDistance)
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
            }

            if (G_set.size() > obstacles.size())
            {
                TRACE_ERROR("ERROR: G_set may not be larger than obstacles!");
            }
            TRACE("Computed step 3. G_set.size() = %d", G_set.size());

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
            float bp_max = 0.0;
            float bm_max = 0.0;
            std::vector<pp_obstacle_struct_t> G_plus;
            std::vector<pp_obstacle_struct_t> G_min;
            std::vector<pp_obstacle_struct_t> G_side_to_use;
            {
                TRACE_SCOPE("cXYTrajectory#Step4", "");
                for (int i = 0; i < (int)G.size(); i++)
                {
                    // Determine if this obstacle belongs in the positive group or negative group.
                    if (G[i].b_i > 0)
                    {
                        G_plus.push_back(G[i]);

                        // Equation (6a)
                        float val = fabs(G[i].b_i + G[i].radius);
                        if (val > bp_max)
                        {
                            bp_max = val;
                        }
                    }
                    else
                    {
                        G_min.push_back(G[i]);

                        // Equation (6b)
                        float val = fabs(G[i].b_i - G[i].radius);
                        if (val > bm_max)
                        {
                            bm_max = val;
                        }
                    }
                }

                //TRACE("b+_max = %6.2f , b-_max = %6.2f", bp_max, bm_max);

                // Equation (6c)
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
            }
            TRACE("Computed step 4.");

            // 5. Compute for each i \in G the angle alpha. Save the i with highest angle -> alpha_max.
            // Determine "j" below equation (8) -> G_idx
            float alpha_max = 0.0;
            float alpha_j = 0.0;
            int G_idx = -1;

            {
                TRACE_SCOPE("cXYTrajectory#Step5", "");
                for (size_t i = 0; i < G_side_to_use.size(); i++)
                {
                    // Determine the sign from equation (7)
                    float sign = 0.0;
                    if ((bm_max - bp_max) < 0)
                        sign = -1.0;
                    else
                        sign = 1.0;

                    // Compute the first and second term of equation (7)
                    float first_term = atan(G_side_to_use[i].b_i / G_side_to_use[i].a_i);
                    float distBetweenObstAndRobot = (Vector2D(G_side_to_use[i].location.x, G_side_to_use[i].location.y)-currPos.xy()).size();
                    float second_term = sign*asin( fmax( fmin( config.subTargetDistance/distBetweenObstAndRobot, 0.99), -0.99) ); // asin's argument must be in [-1 < x < 1]
                    //TRACE("first_term: %6.4f, second_term: %6.4f, sign: %6.4f, distBetweenObstAndRobot: %6.4f", first_term, second_term, sign, distBetweenObstAndRobot);

                    // Equation (7)
                    float alpha = first_term + second_term;
                    float abs_alpha = fabs(alpha);

                    if (abs_alpha > alpha_max)
                    {
                        alpha_max = abs_alpha;
                        alpha_j = alpha;
                        G_idx = i;
                    }

                    //TRACE("(%6.2f,%6.2f) angle is %6.2f", G_side_to_use[i].location.x, G_side_to_use[i].location.y, alpha);
                }
            }

            TRACE("Computed step 5.");

            if (G_idx != -1)
            {
                TRACE_SCOPE("cXYTrajectory#Step6", "");
                // Equation (8) -- G_idx == j
                Vector2D rightPart = ((target.xy() - currPos.xy()) / (target.xy() - currPos.xy()).size()) * (Vector2D(G_side_to_use[G_idx].location.x, G_side_to_use[G_idx].location.y) - currPos.xy()).size();
                Vector2D multipliedMatrix = Vector2D( ( (cos(alpha_j)*rightPart.x) - (sin(alpha_j)*rightPart.y) ) , ( (sin(alpha_j)*rightPart.x) + (cos(alpha_j)*rightPart.y) ) );
                Vector2D subtarget = currPos.xy() + multipliedMatrix;

                //TRACE("subtarget pre-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);
                //TRACE("currPos pre-extend: x %6.2f, y %6.2f", currPos.x, currPos.y);

                // Put the subtarget further away to prevent the robot from slowing down for the subtarget
                subtarget = subtarget + ((subtarget - currPos.xy()).normalize() * config.subTargetExtensionFactor);
                // ROADMAP: replace this trick with velocity control making use of target.velocity

                //TRACE("subtarget post-extend: x %6.2f, y %6.2f", subtarget.x, subtarget.y);

                subTarget.x = subtarget.x;
                subTarget.y = subtarget.y;
                calculatedSubTarget = true;
            }

            TRACE("All done.");
        }
    }

    // done
    if (calculatedSubTarget)
    {
        // add waypoint, leave original target orientation and velocity intact
        data.insertSubTarget(Position2D(subTarget.x, subTarget.y, data.path.at(0).pos.Rz));
        TRACE("Algorithm done, inserted subTarget (%6.2f, %6.2f)", subTarget.x, subTarget.y);
    }
    else
    {
        TRACE("Algorithm done, no obstacle avoidance needed");
    }
}

