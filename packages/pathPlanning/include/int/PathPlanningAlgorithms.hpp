// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanningAlgorithms.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_ALGORITHMS_HPP_
#define PATHPLANNING_ALGORITHMS_HPP_


#include "int/PathPlanningData.hpp"
#include "tracing.hpp"

/*!
 * \brief is the abstract class that each PathPlanning algorithm should inherit.
 *
 */
class PathPlanningAlgorithm
{
public:
    PathPlanningAlgorithm() {}

    virtual void execute(PathPlanningData &data) = 0;
};


// algorithm definitions, each implementation has its own file

class RequireWorldModelActive : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckStopCommand : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckTargetValid : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckTargetReached : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class Shielding : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class ForwardDriving : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CalculateObstacles : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class EscapeForbiddenAreas : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class AvoidObstacles : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

#endif
