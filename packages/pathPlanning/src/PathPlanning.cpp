// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanning.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */
#include <boost/bind.hpp>

// own package
#include "int/PathPlanning.hpp"
#include "int/PathPlanningAlgorithms.hpp"

// other Falcons packages
#include "tracing.hpp"
#include "cDiagnostics.hpp"


PathPlanning::PathPlanning(CFI *configInterface, InputInterface *inputInterface, OutputInterface *outputInterface)
{
    TRACE_FUNCTION("");
    _configInterface = configInterface;
    _inputInterface = inputInterface;
    _outputInterface = outputInterface;
    if (_configInterface != NULL)
    {
        _configInterface->get(data.config);
    }
}

PathPlanning::~PathPlanning()
{
}

actionResultTypeEnum PathPlanning::iterate()
{
    TRACE_FUNCTION("");

    // clear intermediate and output data, retrieve configuration
    prepare();

    // get inputs from input interface
    getInputs();

    // calculate
    data.timestamp = ftime::now();
    auto result = calculate();

    // write outputs to output interface
    setOutputs();

    TRACE("result=%s", enum2str(result));

    // wrap up
    return result;
}

actionResultTypeEnum PathPlanning::calculate()
{
    // this function assumes all inputs are set (see iterate() wrapper)

    TRACE_FUNCTION("");

    data.traceInputs();

    // configure the sequence of algorithms
    std::vector<PathPlanningAlgorithm *> algorithms; // previously known as 'blocks'

    // basics
    algorithms.push_back(new RequireWorldModelActive());
    algorithms.push_back(new CheckStopCommand());
    algorithms.push_back(new CheckTargetValid());
    algorithms.push_back(new CheckTargetReached());

    // subtarget calculation
    algorithms.push_back(new EscapeForbiddenAreas());
    algorithms.push_back(new CalculateObstacles()); // opponents + teammembers + forbiddenAreas + projectedSpeedVectors
    algorithms.push_back(new AvoidObstacles());
    //algorithms.push_back(new Shielding());
    algorithms.push_back(new ForwardDriving());

    // initially, the path consists of just the given target
    data.path.push_back(data.target);

    // execute the sequence of algorithms
    for (auto it = algorithms.begin(); it != algorithms.end(); ++it)
    {
        if (!data.done)
        {
            (*it)->execute(data);
        }
    }

    data.traceOutputs();
    WRITE_TRACE;

    return data.resultStatus;
}

void PathPlanning::getInputs()
{
    TRACE_FUNCTION("");
    // configuration is handled at construction and upon change
    if (_inputInterface != NULL)
    {
        // query RTDB once so we could do repeated gets
        _inputInterface->fetch();
        // get and store data
        motionSetpoint sp = _inputInterface->getMotionSetpoint();
        data.target.pos = pose();
        data.stop = true;
        data.motionType = sp.motionType; 
        if (sp.action == actionTypeEnum::MOVE) // for any other action: do nothing
        {
            data.target.pos.x = sp.position.x;
            data.target.pos.y = sp.position.y;
            data.target.pos.Rz = sp.position.z;
            data.stop = false;
        }
        data.target.vel = pose(); // nonzero input velocity is not yet supported on external interface
        data.forbiddenAreas = _inputInterface->getForbiddenAreas();
        data.addForbiddenAreas(data.forbiddenAreas); // add to calculatedForbiddenAreas
        data.robot = _inputInterface->getRobotState();
        data.teamMembers = _inputInterface->getTeamMembers();
        data.obstacles = _inputInterface->getObstacles();
        data.balls = _inputInterface->getBalls();
    }
}

void PathPlanning::setOutputs()
{
    TRACE_FUNCTION("");
    if (_outputInterface != NULL)
    {
        // Output of PathPlanning is the first wayPoint / subTarget
        robotPosVel subTarget;
        if (data.stop)
        {
            subTarget.robotPosVelType = robotPosVelEnum::VEL_ONLY;
            subTarget.velocity = pose(0.0, 0.0, 0.0);
            subTarget.motionType = motionTypeEnum::NORMAL;
        }
        else
        {
            subTarget.robotPosVelType = robotPosVelEnum::POSVEL;
            subTarget.position = data.path.front().pos;
            subTarget.velocity = data.path.front().vel;
            subTarget.motionType = data.motionType;

            if (data.robot.hasBall)
            {
                subTarget.motionType = motionTypeEnum::WITH_BALL;
            }
        }

        _outputInterface->setSubtarget(data.resultStatus, subTarget);
        _outputInterface->setDiagnostics(makeDiagnostics());
    }
}

diagPathPlanning PathPlanning::makeDiagnostics()
{
    // data adapter
    diagPathPlanning result;
    result.path = data.path;
    result.forbiddenAreas = data.calculatedForbiddenAreas;
    result.distanceToSubTargetRCS.x = data.deltaPositionRcs.x;
    result.distanceToSubTargetRCS.y = data.deltaPositionRcs.y;
    result.distanceToSubTargetRCS.Rz = data.deltaPositionRcs.phi;
    result.numCalculatedObstacles = data.calculatedObstacles.size();
    return result;
}

void PathPlanning::prepare()
{
    TRACE_FUNCTION("");
    data.reset();

    // new configuration?
    if (_configInterface != NULL)
    {
        _configInterface->get(data.config);
    }

    // timestepping
    if (data.config.nominalFrequency > 0)
    {
        data.dt = 1.0 / data.config.nominalFrequency;
    }
    else
    {
        data.dt = 1.0 / 20.0;
    }
    TRACE("nominalFrequency=%.1f dt=%.4fs", data.config.nominalFrequency, data.dt);
}

