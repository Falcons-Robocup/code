// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "MatchSimulationChecks.hpp"

// standard/system headers
// ...


MatchSimulationChecks::MatchSimulationChecks(MatchSimulation &m)
:
    _m(m)
{
}

MatchSimulationChecks::~MatchSimulationChecks()
{
}

void MatchSimulationChecks::setExpectedPosVel(mtp::PlayerId const &playerId, mtp::Pose const &position, mtp::Pose const &velocity)
{
    _expectedPosition[playerId] = position;
    _expectedVelocity[playerId] = velocity;
}

bool MatchSimulationChecks::checkRoleAllocation() const
{
    // checks:
    // 1. each robot must report ready-to-play
    // 2. the role allocation per team must be valid
    // notes:
    // * it can happen that check 1 is true, but 2 not, in case robots are not communicating with each other
    bool result = true;
    std::map<char, mtp::RoleCount> roleCounts;
    for (const auto& p: _m.getPlayers())
    {
        auto robot = _m.getRobot(p);
        if (!robot.readyToPlay()) // check 1
        {
            result = false;
            break;
        }
        roleCounts[robot.id.teamId][robot.getOwnRole()] += 1;
    }
    // check 2
    if (roleCounts.size() < 1) result = false;
    if (roleCounts.size() > 2) result = false;
    for (const auto& count: roleCounts)
    {
        if (!mtp::checkRoleCount(count.second)) result = false;
    }
    return result;
}

bool MatchSimulationChecks::checkRoleAllocation(char teamId, mtp::RoleAllocation const &expectedRoles) const
{
    for (const auto& p: _m.getPlayers())
    {
        auto robot = _m.getRobot(p);
        if (teamId == robot.id.teamId)
        {
            if (!expectedRoles.count(p))
            {
                tprintf("ERROR: expectedRoles is incomplete, missing player %s", p.describe().c_str());
                return false;
            }
            if (robot.getOwnRole() != expectedRoles.at(p)) return false;
        }
    }
    return true;
}


bool MatchSimulationChecks::checkTeamMemberCount(char teamId, int expectedCount) const
{
    int count = 0;
    for (auto &p : _m.getPlayers())
    {
        auto robot = _m.getRobot(p);
        if (robot.id.teamId == teamId)
        {
            count += 1;
            if ((int)robot.getTeam().size() != expectedCount) return false;
        }
    }
    return count == expectedCount;
}

bool MatchSimulationChecks::checkWorldModelPosVel() const
{
    bool result = true;
    for (auto &p : _m.getPlayers())
    {
        auto robot = _m.getRobot(p);
        for (auto &member : robot.getTeam())
        {
            if (member.id == p)
            {
                result = result && (member.position.x == _expectedPosition.at(p).x);
                result = result && (member.position.y == _expectedPosition.at(p).y);
                result = result && (member.velocity.x == _expectedVelocity.at(p).x);
                result = result && (member.velocity.y == _expectedVelocity.at(p).y);
            }
        }
    }
    return result;
}
