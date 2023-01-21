// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BehaviorTreeTypes.hpp
 *
 *  Created on: 2021-01-14
 *      Author: Erik Kouters
 */

#ifndef BEHAVIORTREETYPES_HPP_
#define BEHAVIORTREETYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bimap.hpp>

#include "behaviortree_cpp_v3/basic_types.h"

#include "tracing.hpp"
#include "cDiagnostics.hpp"

// sharedTypes
#include "behTreeReturnEnum.hpp"
#include "motionTypeEnum.hpp"

#include "HeightmapNames.hpp"

#include "int/stores/BallStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"

///////////
// float //
///////////
namespace BT
{
    template <> inline float convertFromString(StringView str)
    {
        return std::stof( (std::string)str );
    }
} 

/////////
// poi //
/////////

// poi = [
//        robot, ball, closestOpponent, closestTeammember, closestAttacker,
//        attackerClosestToOpponentGoal, closestDefender, defenderClosestToOpponentGoal, anyAttacker,
//        anyDefender, myAssistant, goalOpening, coord:2.1,5.8 , fieldPOI
//       ]
enum class tpPOIEnum
{
    INVALID = 0,
        
    ROBOT,
    BALL,
    CLOSEST_OPPONENT,
    CLOSEST_TEAMMEMBER,
    CLOSEST_ATTACKER,
    ATTACKER_CLOSEST_TO_OPPONENT_GOAL,
    ANY_ATTACKER,
    CLOSEST_DEFENDER,
    DEFENDER_CLOSEST_TO_OPPONENT_GOAL,
    ANY_DEFENDER,
    MY_ASSISTANT,
    GOAL_OPENING,

    SIZE_OF_ENUM
};

static std::map<std::string, tpPOIEnum> poiMapping = boost::assign::map_list_of
        ("robot", tpPOIEnum::ROBOT)
        ("ball", tpPOIEnum::BALL)
        ("closestOpponent", tpPOIEnum::CLOSEST_OPPONENT)
        ("closestTeammember", tpPOIEnum::CLOSEST_TEAMMEMBER)
        ("closestAttacker", tpPOIEnum::CLOSEST_ATTACKER)
        ("attackerClosestToOpponentGoal", tpPOIEnum::ATTACKER_CLOSEST_TO_OPPONENT_GOAL)
        ("anyAttacker", tpPOIEnum::ANY_ATTACKER)
        ("closestDefender", tpPOIEnum::CLOSEST_DEFENDER)
        ("defenderClosestToOpponentGoal", tpPOIEnum::DEFENDER_CLOSEST_TO_OPPONENT_GOAL)
        ("goalOpening", tpPOIEnum::GOAL_OPENING)
        ("anyDefender", tpPOIEnum::ANY_DEFENDER)
        ("myAssistant", tpPOIEnum::MY_ASSISTANT)
        ;

namespace BT
{
    template <> inline Point2D convertFromString(StringView str)
    {
        Point2D result;

        try
        {
            auto poi = poiMapping.find( (std::string)str );
            if (poi != poiMapping.end())
            {
                switch(poi->second)
                {
                    case tpPOIEnum::ROBOT:
                    {
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();

                        result.x = myPos.x;
                        result.y = myPos.y;
                        return result;
                    }
                    case tpPOIEnum::BALL:
                    {
                        // If location not known, defaults to lastKnownBallLocation
                        auto ball = teamplay::BallStore::getBall().getPosition();

                        result.x = ball.x;
                        result.y = ball.y;
                        return result;
                        break;
                    }
                    case tpPOIEnum::CLOSEST_OPPONENT:
                    {
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto sortedListOfObstacles = teamplay::ObstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(myPos);

                        if (!sortedListOfObstacles.empty())
                        {
                            result = sortedListOfObstacles.front().getLocation();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve ClosestOpponent. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("ClosestOpponent resolved to ") << result.str();
                        
                        return result;
                        break;
                    }
                    case tpPOIEnum::CLOSEST_TEAMMEMBER:
                    {
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto sortedListOfRobots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(myPos);

                        if (sortedListOfRobots.size() > 1)
                        {
                            // idx 0 == myRobot
                            // idx 1 == closest teammember
                            auto teammemberPos = sortedListOfRobots.at(1).getPosition();
                            result = teammemberPos;
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve ClosestTeammember. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("ClosestTeammember resolved to ") << result.str();
                        
                        return result;
                        break;
                    }
                    case tpPOIEnum::CLOSEST_ATTACKER:
                    case tpPOIEnum::ANY_ATTACKER:       // Default to CLOSEST_ATTACKER for ANY_ATTACKER
                    {
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();
                        auto robots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(myPos);

                        std::vector<teamplay::Robot> attackers;
                        std::copy_if(robots.begin(), robots.end(), std::back_inserter(attackers),
                                     [&](const teamplay::Robot &r) {
                                         const auto role = r.getRole();
                                         return (role != myRole) && ((role == teamplay::RoleEnum::ATTACKER_MAIN) || (role == teamplay::RoleEnum::ATTACKER_ASSIST));
                                     });

                        if (!attackers.empty())
                        {
                            result = attackers.front().getPosition();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve ClosestAttacker/AnyAttacker. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("ClosestAttacker/AnyAttacker resolved to ") << result.str();

                        return result;
                        break;
                    }
                    case tpPOIEnum::ATTACKER_CLOSEST_TO_OPPONENT_GOAL:
                    {
                        auto oppGoalPos = teamplay::FieldDimensionsStore::getFieldDimensions().getLocation( teamplay::FieldPOI::OPP_GOALLINE_CENTER );
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();
                        auto robots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(oppGoalPos);

                        std::vector<teamplay::Robot> attackers;
                        std::copy_if(robots.begin(), robots.end(), std::back_inserter(attackers),
                                     [&](const teamplay::Robot &r) {
                                         const auto role = r.getRole();
                                         return (role != myRole) && ((role == teamplay::RoleEnum::ATTACKER_MAIN) || (role == teamplay::RoleEnum::ATTACKER_ASSIST));
                                     });

                        if (!attackers.empty())
                        {
                            result = attackers.front().getPosition();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve AttackerClosestToOpponentGoal. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("AttackerClosestToOpponentGoal resolved to ") << result.str();

                        return result;
                        break;
                    }
                    case tpPOIEnum::CLOSEST_DEFENDER:
                    case tpPOIEnum::ANY_DEFENDER:       // Default to CLOSEST_DEFENDER for ANY_DEFENDER
                    {
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();
                        auto robots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(myPos);

                        std::vector<teamplay::Robot> defenders;
                        std::copy_if(robots.begin(), robots.end(), std::back_inserter(defenders),
                                     [&](const teamplay::Robot &r) {
                                         const auto role = r.getRole();
                                         return (role != myRole) && ((role == teamplay::RoleEnum::DEFENDER_MAIN) || (role == teamplay::RoleEnum::DEFENDER_ASSIST));
                                     });

                        if (!defenders.empty())
                        {
                            result = defenders.front().getPosition();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve ClosestDefender/AnyDefender. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("ClosestDefender/AnyDefender resolved to ") << result.str();

                        return result;
                        break;
                    }
                    case tpPOIEnum::DEFENDER_CLOSEST_TO_OPPONENT_GOAL:
                    {
                        auto oppGoalPos = teamplay::FieldDimensionsStore::getFieldDimensions().getLocation( teamplay::FieldPOI::OPP_GOALLINE_CENTER );
                        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();
                        auto robots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(oppGoalPos);

                        std::vector<teamplay::Robot> defenders;
                        std::copy_if(robots.begin(), robots.end(), std::back_inserter(defenders),
                                     [&](const teamplay::Robot &r) {
                                         const auto role = r.getRole();
                                         return (role != myRole) && ((role == teamplay::RoleEnum::DEFENDER_MAIN) || (role == teamplay::RoleEnum::DEFENDER_ASSIST));
                                     });

                        if (!defenders.empty())
                        {
                            result = defenders.front().getPosition();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve DefenderClosestToOpponentGoal. Falling back to my position.");
                            result = myPos;
                        }

                        TRACE("DefenderClosestToOpponentGoal resolved to ") << result.str();

                        return result;
                        break;
                    }
                    case tpPOIEnum::MY_ASSISTANT:
                    {
                        auto myAssistant = teamplay::RobotStore::getInstance().getAssistantOfOwnRobot();

                        if (myAssistant)
                        {
                            result = myAssistant->getPosition();
                        }
                        else
                        {
                            // fallback: myPos
                            TRACE_ERROR("WARNING: Failed to resolve MyAssistant. Falling back to my position.");
                            result = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                        }

                        TRACE("MyAssistant resolved to ") << result.str();
                        
                        return result;
                        break;
                    }
                    case tpPOIEnum::GOAL_OPENING:
                    {
                        // In meters; amount 
                        static float GOALPOST_CORRECTION = 0.3f;

                        auto fieldDimensions = teamplay::FieldDimensionsStore::getFieldDimensions();
                        auto oppGoalCenter = fieldDimensions.getLocation(teamplay::FieldPOI::OPP_GOALLINE_CENTER);

                        // TODO: instead trace lines from robot to oppenents and see if they hit goal-line, then shoot somewhere in between.
                        // TODO: add condition node to see if there is an opnening at all, so we can pass to AA instead
                        // TODO: take keeper velocity into account
                        auto sortedListOfObstacles = teamplay::ObstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(oppGoalCenter);
                        if (!sortedListOfObstacles.empty())
                        {
                            // assume closest obstacle to goal is keeper
                            const auto& keeper = sortedListOfObstacles.front();

                            if (fieldDimensions.isPositionInOpponentGoalArea(keeper.getLocation().x, keeper.getLocation().y))
                            {
                                auto oppGoalLeft = fieldDimensions.getLocation(teamplay::FieldPOI::OPP_GOALPOST_LEFT);
                                auto oppGoalRight = fieldDimensions.getLocation(teamplay::FieldPOI::OPP_GOALPOST_RIGHT);

                                if (keeper.getLocation().x < 0)
                                {   // keeper is in left side of the goal, so shoot at right side
                                    result = oppGoalRight;
                                    result.x -= GOALPOST_CORRECTION;
                                }
                                else
                                {   // keeper is in right side of the goal, so shoot at left side
                                    result = oppGoalLeft;
                                    result.x += GOALPOST_CORRECTION;
                                }
                            }
                            else 
                            {
                                // If the "keeper" is not in the opponent goal area, still shoot in the middle.
                                // If we don't see the keeper, we may base our decision on an opponent that is irrelevant.
                                result = oppGoalCenter;
                            }
                        }
                        else
                        {
                            result = oppGoalCenter;
                        }                     
                        TRACE("GoalOpening resolved to ") << result.str();

                        return result;
                        break;
                    }
                    default:
                    {
                        throw BT::RuntimeError("Error: unknown tpPOIEnum found");
                        break;
                    }
                }
            }
            else if (boost::starts_with( (std::string)str, "coord:"))
            { //expected syntax example:  coord 4.5,6.8
                std::vector<std::string> splittedString;
                boost::split(splittedString, (std::string)str, boost::is_any_of(":,"));
                if (splittedString.size() != 3)
                {
                    throw std::runtime_error("MALFORMED coord parameter encountered: " + (std::string)str);
                }

                try
                {
                    result.x = boost::lexical_cast<double>(splittedString[1]);
                }
                catch (std::exception &e)
                {
                    throw std::runtime_error("MALFORMED coord parameter encountered for X in: " + (std::string)str);
                }

                try
                {
                    result.y = boost::lexical_cast<double>(splittedString[2]);
                }
                catch (std::exception &e)
                {
                    throw std::runtime_error("MALFORMED coord parameter encountered for Y in: " + (std::string)str);
                }
                return result;
            }
            else
            {
                // str may be a field POI
                result = teamplay::FieldDimensionsStore::getFieldDimensions().getLocation( (std::string)str );
            }
        }
        catch(std::exception& e)
        {
            throw std::runtime_error("Failed to parse Point2D from string '" + (std::string)str + "'");
        }

        return result;
    }
} 



////////////
// Area2D //
////////////

// FieldArea is defined in types/FieldDimensions.hpp

namespace BT
{
    template <> inline Area2D convertFromString(StringView str)
    {
        Area2D result;
        if ( teamplay::FieldDimensionsStore::getFieldDimensions().isValidArea( (std::string)str ) )
        {
            result = teamplay::FieldDimensionsStore::getFieldDimensions().getArea( (std::string)str );
        }
        else
        {
            throw std::runtime_error("Failed to parse Area2D from string '" + (std::string)str + "'");
        }
        return result;
    }
} 



/////////////////////
// tpShootTypeEnum //
/////////////////////

enum class tpShootTypeEnum
{
    INVALID = 0,

    SHOOT,
    LOB_SHOT,
    PASS,

    SIZE_OF_ENUM
};

static boost::bimap<std::string, tpShootTypeEnum> shootTypeMapping = boost::assign::list_of< boost::bimap<std::string, tpShootTypeEnum>::relation >
        ("shoot", tpShootTypeEnum::SHOOT)
        ("lobShot", tpShootTypeEnum::LOB_SHOT)
        ("pass", tpShootTypeEnum::PASS)
        ;

namespace BT
{
    template <> inline tpShootTypeEnum convertFromString(StringView str)
    {
        return shootTypeMapping.left.at( (std::string)str );
    }
} 



////////////////////
// motionTypeEnum //
////////////////////

// motionTypeEnum is defined in sharedTypes

static boost::bimap<std::string, motionTypeEnum> motionTypeEnumMapping = boost::assign::list_of< boost::bimap<std::string, motionTypeEnum>::relation >
        ("normal", motionTypeEnum::NORMAL)
        ("with_ball", motionTypeEnum::WITH_BALL)
        ("accurate", motionTypeEnum::ACCURATE)
        ("intercept", motionTypeEnum::INTERCEPT)
        ("slow", motionTypeEnum::SLOW)
        ("sprint", motionTypeEnum::SPRINT)
        ;

namespace BT
{
    template <> inline motionTypeEnum convertFromString(StringView str)
    {
        return motionTypeEnumMapping.left.at( (std::string)str );
    }
} 

//////////////
// RoleEnum //
//////////////

// RoleEnum is defined in types/Role.hpp

static boost::bimap<std::string, teamplay::RoleEnum> roleEnumMapping = boost::assign::list_of< boost::bimap<std::string, teamplay::RoleEnum>::relation >
        ("goalkeeper", teamplay::RoleEnum::GOALKEEPER)
        ("attackerMain", teamplay::RoleEnum::ATTACKER_MAIN)
        ("attackerAssist", teamplay::RoleEnum::ATTACKER_ASSIST)
        ("defenderMain", teamplay::RoleEnum::DEFENDER_MAIN)
        ("defenderAssist", teamplay::RoleEnum::DEFENDER_ASSIST)
        ;

namespace BT
{
    template <> inline teamplay::RoleEnum convertFromString(StringView str)
    {
        return roleEnumMapping.left.at( (std::string)str );
    }
} 



////////////////////////////
// CompositeHeightmapName //
////////////////////////////

// CompositeHeightmapName is defined in HeightmapNames.hpp

static boost::bimap<std::string, CompositeHeightmapName> heightmapEnumMapping = boost::assign::list_of< boost::bimap<std::string, CompositeHeightmapName>::relation >
        ("defendAttackingOpponent", CompositeHeightmapName::DEFEND_ATTACKING_OPPONENT)
        ("dribble", CompositeHeightmapName::DRIBBLE)
        ("moveToFreeSpot", CompositeHeightmapName::MOVE_TO_FREE_SPOT)
        ("positionAttackerForOwnSetpiece", CompositeHeightmapName::POSITION_ATTACKER_FOR_OWN_SETPIECE)
        ("positionForOppSetpiece", CompositeHeightmapName::POSITION_FOR_OPP_SETPIECE)
        ;

namespace BT
{
    template <> inline CompositeHeightmapName convertFromString(StringView str)
    {
        return heightmapEnumMapping.left.at( (std::string)str );
    }
} 


//////////////////////
// SetpieceTreeTypeEnum //
//////////////////////

enum class SetpieceTreeTypeEnum
{
    INVALID = 0,

    ANY,
    OWN,
    PREPARE,
    KICKOFF,
    CORNER,
    THROWIN,
    GOALKICK,
    FREEKICK,
    DROPPED_BALL,
    PENALTY,
    PARK,
    SHOWING_ALIVE,

    SIZE_OF_ENUM
};

static boost::bimap<std::string, SetpieceTreeTypeEnum> setpieceTreeTypeEnumMapping = boost::assign::list_of< boost::bimap<std::string, SetpieceTreeTypeEnum>::relation >
        ("Any", SetpieceTreeTypeEnum::ANY)
        ("Own", SetpieceTreeTypeEnum::OWN)
        ("Prepare", SetpieceTreeTypeEnum::PREPARE)
        ("Kickoff", SetpieceTreeTypeEnum::KICKOFF)
        ("Corner", SetpieceTreeTypeEnum::CORNER)
        ("Throwin", SetpieceTreeTypeEnum::THROWIN)
        ("Goalkick", SetpieceTreeTypeEnum::GOALKICK)
        ("Freekick", SetpieceTreeTypeEnum::FREEKICK)
        ("DroppedBall", SetpieceTreeTypeEnum::DROPPED_BALL)
        ("Penalty", SetpieceTreeTypeEnum::PENALTY)
        ("Park", SetpieceTreeTypeEnum::PARK)
        ("ShowingAlive", SetpieceTreeTypeEnum::SHOWING_ALIVE)
        ;

namespace BT
{
    template <> inline SetpieceTreeTypeEnum convertFromString(StringView str)
    {
        return setpieceTreeTypeEnumMapping.left.at( (std::string)str );
    }
} 



///////////////////////
// behTreeReturnEnum //
///////////////////////

static boost::bimap<behTreeReturnEnum, BT::NodeStatus> nodeStatusMapping = boost::assign::list_of< boost::bimap<behTreeReturnEnum, BT::NodeStatus>::relation >
        (behTreeReturnEnum::INVALID, BT::NodeStatus::IDLE)
        (behTreeReturnEnum::PASSED, BT::NodeStatus::SUCCESS)
        (behTreeReturnEnum::FAILED, BT::NodeStatus::FAILURE)
        (behTreeReturnEnum::RUNNING, BT::NodeStatus::RUNNING)
        ;

static boost::bimap<std::string, behTreeReturnEnum> behTreeReturnEnumMapping = boost::assign::list_of< boost::bimap<std::string, behTreeReturnEnum>::relation >
        ("INVALID", behTreeReturnEnum::INVALID)
        ("PASSED", behTreeReturnEnum::PASSED)
        ("FAILED", behTreeReturnEnum::FAILED)
        ("RUNNING", behTreeReturnEnum::RUNNING)
        ;

#endif /* CDECISIONTREETYPES_HPP_ */
