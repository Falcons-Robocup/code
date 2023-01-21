// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "MatchSimulation.hpp"

// header from MTP
#include "Roles.hpp"

// standard/system headers
// ...


MatchSimulation::MatchSimulation(float frequency)
:
    _t0(rtime::now()),
    _tc(_t0),
    _tstep(1.0 / frequency)
{
}

MatchSimulation::~MatchSimulation()
{
}

bool operator<(RobotClient const &a, RobotClient const &b)
{
    if (a.id.teamId == b.id.teamId) return a.id.shirtId < b.id.shirtId;
    return a.id.teamId < b.id.teamId;
}

RobotClient &MatchSimulation::addRobot(mtp::PlayerId const &playerId, float frequency, float jitter)
{
    if (_robots.count(playerId)) throw std::runtime_error("player already registered: " + playerId.describe());
    _robots.try_emplace(playerId, RobotClient(playerId, _t0, frequency, jitter));
    return getRobot(playerId);
}

RobotClient& MatchSimulation::getRobot(mtp::PlayerId const &playerId)
{
    try
    {
        return _robots.at(playerId);
    }
    catch (...)
    {
        throw std::runtime_error("player not registered: " + playerId.describe());
    }
}

void MatchSimulation::synchronize()
{
    // normally, on robot (or possibly also in simulation) the Comm process/thread takes care of synchronizing data
    // however in this test suite we simulate without Comm and without its sockets and sleeps
    // so a database synchronization mechanism is needed, currently not provided by RTDB/COMM API

    // previously, a symlink trick was applied to realize this
    // but it has a negative side effect that within a tick robots incrementally know each others data, limiting the tests we could do here

    for (auto& robot: _robots)
    {
        // mimick part of Comm (TODO: refactor Comm/API to provide this core functionality?)

        // get the frame string of current robot
        std::string frameString = robot.second.getFrameString();
        // give data to friendly robots
        for (auto& otherRobot: _robots)
        {
            if (robot.first.teamId == otherRobot.first.teamId && otherRobot.first.hash() != robot.first.hash())
            {
                otherRobot.second.setFrameString(frameString);
            }
        }
    }
}

void MatchSimulation::advanceTick()
{
    if (_verbose && _tc == _t0) reportHeading();
    _tc += _tstep;
    // synchronize data between robots
    synchronize();
    // poke robots
    for (auto& robot: _robots)
    {
        robot.second.tick(_tc);
    }
    if (_verbose) reportTick();
}

void MatchSimulation::advanceTicks(int ticks)
{
    while (ticks--) advanceTick();
}

void MatchSimulation::advanceDuration(float duration)
{
    rtime te = _tc + duration;
    while (_tc < te) advanceTick();
}

void MatchSimulation::reportHeading() const
{
    printf("\nSimulated robots at t=0:\n");
    for (const auto& robot: _robots)
    {
        printf("   %s\n", robot.second.statusReportLong().c_str());
    }
    printf("\n");
    printf("time (s)  ");
    for (const auto& robot: _robots)
    {
        printf("%-10d", robot.second.id.hash());
    }
    printf("\n");
}

void MatchSimulation::reportTick() const
{
    /* Legend:
    X: robot reports not ready
    L: robot claims to be the leader
    C: robot current role has changed 
    P: robot has a preference which is satisfied OK
    Q: robot has a preference but MTP rejects it
    */
    printf("%8.3f  ", (double)(_tc - _t0));
    for (const auto& robot: _robots)
    {
        printf("%-10s", robot.second.statusReportBrief().c_str());
    }
    printf("\n");
}

std::vector<mtp::PlayerId> MatchSimulation::getPlayers() const
{
    std::vector<mtp::PlayerId> result;
    for (const auto& robot: _robots) result.push_back(robot.first);
    return result;
}
