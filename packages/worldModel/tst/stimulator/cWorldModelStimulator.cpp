// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModelStimulator.cpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 */


// system includes
#include <cstdio>

// other Falcons packages
#include "falconsCommon.hpp"
#include "tracing.hpp"

// internal includes
#include "cWorldModelStimulator.hpp"

cWorldModelStimulator::cWorldModelStimulator(int agentId, std::string inputFile, std::string outputFile)
{
    _inputFile = inputFile;
    _outputFile = outputFile;
    _agentId = agentId; // TODO I think we can strip this -- script sets env, wm knows itself which agent it is
    _component = "worldModel";
    _overruleRobotPos = false;
    INIT_TRACE;
}

cWorldModelStimulator::~cWorldModelStimulator()
{
}

bool cWorldModelStimulator::checkFrame(tLogFrame const &frame)
{
    return true; // TODO OK?
    /*
    // we don't introspect the data, we use the fact that the logger writes per agent a local and a shared frame on some frequency
    // so whenever we see such a frame, we have to recalculate
    if ((_agentId == frame.agent) && (frame.shared == true))
    {
        // note: here we could store things for use in tick, if we'd need to
        return true;
    }
    return false;
    */
}

void cWorldModelStimulator::overruleRobotPos()
{
    T_ROBOT_STATE robotRtdb;
    int r = _rtdb->get(ROBOT_STATE, &robotRtdb, _agentId);
    if (r == RTDB2_SUCCESS)
    {
        std::cout << "overrulling" << std::endl;
        // convert to wm-internal struct robotClass_t
        robotClass_t robot;
        robot.setRobotID(_agentId);
        robot.setTimestamp(robotRtdb.timestamp);
        robot.setCoordinateType(coordinateType::FIELD_COORDS); // unused, TODO remove
        robot.setCoordinates(robotRtdb.position.x, robotRtdb.position.y, robotRtdb.position.Rz);
        robot.setVelocities(robotRtdb.velocity.x, robotRtdb.velocity.y, robotRtdb.velocity.Rz);
        robot.setBallPossession(robotRtdb.hasBall);
        
        _worldModel.getRobotAdministrator()->updateRobotPositionAndVelocity(robot);
    }
}

void cWorldModelStimulator::tick(rtime const &t)
{
    if(_overruleRobotPos)
    {
        overruleRobotPos();
    }        

    _worldModel.update(t);
    if (_verbosity >= 3)
    {
        diagWorldModel wmDiag = _worldModel.getDiagnostics();
        // TODO tprintf
        //printf("  inplay=%d active=[%s] #vis=%2d #motor=%2d #balls=%2d #obst=%2d", 
        //    wmDiag.inplay, wmDiag.teamActivity.c_str(), wmDiag.numVisionCandidates, wmDiag.numMotorDisplacementSamples, wmDiag.numBallTrackers, wmDiag.numObstacleTrackers);
    }
}

void cWorldModelStimulator::setRobotPosOverrule(bool overrule)
{
    _overruleRobotPos = overrule;
}

