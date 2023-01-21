// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDBInputAdapter.hpp"

#include "int/gamestate/GameStateManager.hpp"
#include "int/rules/RuleStimulatePassing.hpp"
#include "int/stores/BallStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"

#include "cDiagnostics.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

tpRTDBInputAdapter::tpRTDBInputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
    TRACE("<");
}

void tpRTDBInputAdapter::waitForTPHeartbeat(void (*iterateTeamplayFuncPtr) (void)) const
{
    while (true)
    {
        _rtdb->waitForPut(TP_HEARTBEAT);

        iterateTeamplayFuncPtr();
    }
}

void tpRTDBInputAdapter::waitForRobotRoles(void (*iterateTeamplayFuncPtr) (void)) const
{
    while (true)
    {
        _rtdb->waitForPut(ROBOT_ROLES);

        iterateTeamplayFuncPtr();
    }
}

void tpRTDBInputAdapter::getWorldModelData()
{
    getWorldModelData(_myRobotId);
}

void tpRTDBInputAdapter::setMTP(MixedTeamProtocolAdapter* mtp)
{
    _mtpAdapter = mtp;
}

void tpRTDBInputAdapter::setWorldModelRobotId(const int robotID)
{
    _wmClient.setRobotId(robotID);
}

void tpRTDBInputAdapter::getWorldModelData(const int robotID)
{
    TRACE_FUNCTION("");

    if (!_mtpAdapter)
    {
        return;
    }

    try
    {
        /* Poke WorldModel to update */
        _wmClient.update(robotID);

        // get teammembers from MTP
        auto mtpTeamMembers = (*_mtpAdapter)->getTeam(false); // TODO revise/clarify API

        /* Clear the stores */
        teamplay::BallStore::getBall().reset();
        teamplay::ObstacleStore::getInstance().clear();
        teamplay::RobotStore::getInstance().clear();

        /* Iterate over all robots to add them to the stores */
        bool teamHasBall = false;
        for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
        {
            TRACE_SCOPE("ITERATE", std::to_string(agentId).c_str());
            robotState wmRobotData;
            if (_wmClient.getRobotState(wmRobotData, agentId, robotID))
            {
                TRACE_SCOPE("HASSTATE", std::to_string(agentId).c_str());
                const geometry::Pose2D tpRobotDataPos = geometry::Pose2D(wmRobotData.position.x, wmRobotData.position.y, wmRobotData.position.Rz);
                const geometry::Velocity2D tpRobotDataVel = geometry::Velocity2D(wmRobotData.velocity.x, wmRobotData.velocity.y, wmRobotData.velocity.Rz);

                auto tpRobotData = teamplay::Robot(wmRobotData.robotId, teamplay::RoleEnum::STOP, tpRobotDataPos, tpRobotDataVel);

                /* Robot role */
                auto leaderRobotNumber = _wmClient.getLowestActiveRobotID();
                auto roles = getRobotRoles(leaderRobotNumber);
                auto it = roles.find(wmRobotData.robotId);
                if (it != roles.end())
                {
                    tpRobotData.setRole( it->second );
                }

                /* Ball possession */
                if (wmRobotData.hasBall)
                {
                    teamHasBall = true;
                    tpRobotData.claimsBallPossession();
                    teamplay::BallStore::getBall().setPositionClaimed(Point3D(wmRobotData.ballAcquired.x, wmRobotData.ballAcquired.y, 0.0));
                    teamplay::RuleStimulatePassing::getInstance().robotClaimsBall(wmRobotData.robotId);
                }

                /* add own_robot / teammate to the stores */
                if (agentId == robotID)
                {
                    /* own Robot */
                    teamplay::RobotStore::getInstance().addOwnRobot(tpRobotData);

                    /* Write our robotdata to MTP */
                    (*_mtpAdapter)->setOwnPosVel(tpRobotDataPos.x, tpRobotDataPos.y, tpRobotDataPos.Rz,
                                                 tpRobotDataVel.x, tpRobotDataVel.y, tpRobotDataVel.Rz,
                                                 1.0);
                    (*_mtpAdapter)->setOwnBallPossession(tpRobotData.hasBall());
                }
                else
                {
                    TRACE_SCOPE("TP2", std::to_string(agentId).c_str());
                    /* teammate */
                    teamplay::RobotStore::getInstance().addTeammate(tpRobotData);
                }
            }
        }

        // handle any MTP team members -- for now only the ones which are not handled above already
        for (auto const &teamMember: mtpTeamMembers)
        {
            if (teamMember.id.vendorId != 74) // TODO: handle in API getTeam? or split it?
            {
                TRACE_SCOPE("MTP", teamMember.id.describe().c_str());
                // TODO: this for now only works with non-conflicting id's ... to be reworked
                int robotId = teamMember.id.shirtId;

                const geometry::Pose2D tpRobotDataPos = geometry::Pose2D(teamMember.position.x, teamMember.position.y, teamMember.position.rz);
                const geometry::Velocity2D tpRobotDataVel = geometry::Velocity2D(teamMember.velocity.x, teamMember.velocity.y, teamMember.velocity.rz);
                
                auto tpRobotData = teamplay::Robot(robotId, teamplay::RoleEnum::STOP, tpRobotDataPos, tpRobotDataVel);

                // ball possession
                if (teamMember.hasBall)
                {
                    teamHasBall = true;
                    tpRobotData.claimsBallPossession();
                    //teamplay::BallStore::getBall().setPositionClaimed(Point3D(wmRobotData.ballAcquired.x, wmRobotData.ballAcquired.y, 0.0));
                    //teamplay::RuleStimulatePassing::getInstance().robotClaimsBall(wmRobotData.robotId);
                }

                // HACK: fill in MTP role

                //enum class RoleEnum
                //{
                //    UNDEFINED = 0,
                //    GOALKEEPER = 1,
                //    ATTACKER_MAIN = 2,
                //    ATTACKER_ASSIST = 3,
                //    ATTACKER_GENERIC = 4,
                //    DEFENDER_MAIN = 5,
                //    DEFENDER_GENERIC = 6,
                //    DISABLED_OUT = 7,
                //    DISABLED_IN = 8
                //}; // end of enum class RoleEnum

                if (teamMember.role == "UNDEFINED")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::INVALID );
                }
                else if (teamMember.role == "GOALKEEPER")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::GOALKEEPER );
                }
                else if (teamMember.role == "ATTACKER_MAIN")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::ATTACKER_MAIN );
                }
                else if (teamMember.role == "ATTACKER_ASSIST")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::ATTACKER_ASSIST );
                }
                else if (teamMember.role == "ATTACKER_GENERIC")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::ATTACKER_ASSIST ); // <-- TODO: Add generic attacker?
                }
                else if (teamMember.role == "DEFENDER_MAIN")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::DEFENDER_MAIN );
                }
                else if (teamMember.role == "DEFENDER_GENERIC")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::DEFENDER_ASSIST );
                }
                else if (teamMember.role == "DISABLED_OUT")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::STOP );
                }
                else if (teamMember.role == "DISABLED_IN")
                {
                    tpRobotData.setRole( teamplay::RoleEnum::STOP );
                }

                teamplay::RobotStore::getInstance().addTeammate(tpRobotData);
            }
        }

        if (!teamHasBall)
        {
            teamplay::BallStore::getBall().setPositionClaimedUnknown();
        }

        /* Store the obstacles */
        auto obstacles = _wmClient.getObstacles();
        for (auto Obstacle : obstacles)
        {
            const geometry::Pose2D pos(Obstacle.position.x, Obstacle.position.y, 0.0);
            const geometry::Velocity2D vel(Obstacle.velocity.x, Obstacle.velocity.y, 0.0);

            teamplay::ObstacleStore::getInstance().addObstacle(teamplay::Obstacle(pos, vel));
        }

        /* Store the Ball */
        auto balls = _wmClient.getBalls();
        if (balls.size() == 0)
        {
            teamplay::BallStore::getBall().setPositionUnknown();
            teamplay::BallStore::getBall().setVelocityUnknown();
        }
        else
        {
            // expect best Ball to be the first in this array
            auto b = balls[0];
            teamplay::BallStore::getBall().setPosition( Point3D(b.position.x, b.position.y, b.position.z) );
            teamplay::BallStore::getBall().setVelocity( Vector3D(b.velocity.x, b.velocity.y, b.velocity.z) );

            /* Write our ball to MTP */
            (*_mtpAdapter)->setOwnBall(b.position.x, b.position.y, b.position.z,
                                       b.velocity.x, b.velocity.y, b.velocity.z,
                                       b.confidence);
        }

    }
    catch (std::exception& e)
    {
        TRACE_ERROR("Caught exception while storing worldmodel data: %s", e.what());
        throw std::runtime_error(std::string("Caught exception while storing worldmodel data: ") + e.what());
    }
}

std::map<teamplay::RobotNumber,teamplay::RoleEnum> tpRTDBInputAdapter::getRobotRoles(const int leaderRobotID) const
{
    T_ROBOT_ROLES robotRoles;

    auto r = _rtdb->get(ROBOT_ROLES, &robotRoles, leaderRobotID);
    if (r != RTDB2_SUCCESS)
    {
        tprintf("rtdb/tp/getRoles failure");
    }

    std::map<teamplay::RobotNumber, teamplay::RoleEnum> result;

    for (const auto& robotRole: robotRoles)
    {
        result.insert( {(teamplay::RobotNumber)robotRole.first, teamplay::str2enum(robotRole.second)} );
    }

    return result;
}

T_ACTION_RESULT tpRTDBInputAdapter::getActionResult() const
{
    T_ACTION_RESULT result = {actionResultTypeEnum::RUNNING};

    auto r = _rtdb->get(ACTION_RESULT, &result);
    if (r != RTDB2_SUCCESS)
    {
        tprintf("rtdb/tp/getActionResult failure");
    }

    return result;
}

T_INTENTION tpRTDBInputAdapter::getIntention(const int robotID) const
{
    T_INTENTION intention;

    auto r = _rtdb->get(INTENTION, &intention, robotID);
    if (r != RTDB2_SUCCESS)
    {
        tprintf("rtdb/tp/getIntention failure for robotID '%d'", robotID);
    }

    return intention;
}
