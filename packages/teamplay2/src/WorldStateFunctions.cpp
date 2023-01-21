// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldStateFunctions.cpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Ivo Matthijssen
 */


#include "int/WorldStateFunctions.hpp"

#include "int/stores/RobotStore.hpp"
#include "int/stores/ObstacleStore.hpp"

#include "falconsCommon.hpp" // for ROBOT_RADIUS

using namespace teamplay;


WorldStateFunctions::WorldStateFunctions()
{
}

void WorldStateFunctions::getObstructingObstaclesInPath(const Point2D robotPos, const Point2D targetPos, const float radiusObjectMeters, std::vector<Obstacle> &obstacles)
{
    /*
     * Steps:
     * 1) Clear vector for consistent output
     * 2) Filter out friendly objects that are in reach of target position
     * 3) Add Obstacle objects in set
     * 4) Calculate group B of pathplanning XY algorithm for Obstacle avoidance
     */
    try
    {
        std::vector<Obstacle> inputSet;

        // Step 1
        obstacles.clear();

        // Step 2
        auto teammembers = teamplay::RobotStore::getInstance().getAllRobotsExclOwnRobot();

        for (const Robot& teammember: teammembers)
        {
            // ?????
            Point2D teammemberPos = teammember.getLocation();
            if(!( ((teammemberPos - targetPos).size() < radiusObjectMeters) ||
                  ((teammemberPos - robotPos).size() < radiusObjectMeters) )
              )
            {
                Obstacle obst = Obstacle(teammember.getPosition(), teammember.getVelocity());
                inputSet.push_back(obst);
            }

        }

        // Step 3 Add all obstacles
        auto opponents = teamplay::ObstacleStore::getInstance().getAllObstacles();
        for (const Obstacle& obst: opponents)
        {
            inputSet.push_back(obst);
        }

        // Step 4 Calculate set B
        // 1. Determine B = Set of all obstacles in the direct path from Robot position to target position, taking also the diameter of the obstacles and the Robot into account.
        for(size_t i = 0; i < inputSet.size(); i++)
        {
            double a_i;
            double b_i;

            // Pre-compute a_i and b_i for all obstacles, and store in the struct for later reuse.
            Point2D obstVec = inputSet.at(i).getLocation();
            a_i = ((targetPos - robotPos) * (obstVec - robotPos)) / (targetPos - robotPos).size();
            b_i = ((targetPos - robotPos).CrossProduct((obstVec - robotPos))) / (targetPos - robotPos).size();

            // Equation (4) from the paper.
            if ( ( 0 < a_i ) &&
                 ( a_i <= (targetPos - robotPos).size() ) &&
                 ( fabs(b_i) < (ROBOT_RADIUS + radiusObjectMeters) )) // r_r + r_i : Robot radius + Obstacle radius
            {
                obstacles.push_back(inputSet.at(i));
            }

        }

        // Sort the Obstacle on nearest to robotPos
        std::sort(obstacles.begin(), obstacles.end(),
                [&](const Obstacle& first, const Obstacle& second)
                { return first.getDistanceTo(robotPos) < second.getDistanceTo(robotPos); });

    } catch (std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

}

