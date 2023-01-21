// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef SIMPLEQUERIES_HPP
#define SIMPLEQUERIES_HPP

#include "behaviortree_cpp_v3/bt_factory.h"

class SimpleQueryControlNode : public BT::ControlNode
{
  public:
    SimpleQueryControlNode(const std::string& name, const BT::NodeConfiguration& config, const std::function<bool()>& simplequery_functor)
      : BT::ControlNode(name, config), _simplequery_functor(simplequery_functor)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {  };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:
    std::function<bool()> _simplequery_functor;
    bool _previousResult = false;

};

void registerNodes_SimpleQueries(BT::BehaviorTreeFactory& btFactory);

///////////////////
// ACTIVE_ROBOTS //
///////////////////
bool isLowestActiveRobotID();           // <-- used in gamestates.json
bool isHighestActiveRobotID();          // <-- used in gamestates.json
bool isSecondHighestActiveRobotID();    // <-- used in gamestates.json
bool isThirdHighestActiveRobotID();     // <-- used in gamestates.json
bool isOnlyActiveRobotID();             // <-- used in setpiece.json (penalty execute)

//////////
// BALL //
//////////
bool isBallLocationKnown();         // <-- used in setpiece.json
bool within1mOfBall();              // <-- used in setpiece.json
bool doesTeamHaveBall();            // <-- used in roles.json
//bool isBallAtSide();    replaced by IsPOIInArea            // <-- used in setpiece.json (ownCornerPrepare)

///////////////
// OWN_ROBOT //
///////////////
//bool isOwnRobotAtOpponentSide();  replaced by IsPOIInArea
//bool isOwnRobotClosestToPOI();                   replaced by IsOwnRobotClosestToPOI     // <-- used in gamestates.json
//bool isOwnRobotSecondClosestToPOI();             replaced by IsOwnRobotClosestToPOI     // <-- used in gamestates.json
//bool isOwnRobotFurthestFromPOI();                replaced by IsOwnRobotClosestToPOI     // <-- used in gamestates.json
//bool isOwnRobotNearestToBall();                  replaced by IsOwnRobotClosestToPOI   // <-- used in setpiece.json + roles.json
//bool isOwnRobotNearestToLastKnownBallLocation(); replaced by IsOwnRobotClosestToPOI   // <-- used in setpiece.json
bool doesOwnRobotHaveBall();                        // <-- used in roles.json (attacker main)
bool isPassApproachingRobot();              // <-- used in roles.json (attackerassist)
bool isBallApproachingRobot();      // <-- used in roles.json (attacker assist)

////////////////
// TEAMMEMBER //
////////////////
bool isAssistantPresent();                      // <-- used in setpiece.json + behaviors.json
bool doesAssistantHaveBall();
//bool isPassToClosestTeammemberBlocked();   replaced by IsShootToPOIBlocked     // <-- used in setpiece.json (ownFreekickExecute)
//bool isPassToClosestAttackerBlocked();     replaced by IsShootToPOIBlocked     // <-- used in setpiece.json (ownFreekickExecute)
//bool isPassToFurthestAttackerBlocked();    replaced by IsShootToPOIBlocked     // <-- used in setpiece.json (ownCornerExecute) (ownFreekickExecute)
//bool isPassToFurthestDefenderBlocked();    replaced by IsShootToPOIBlocked     // <-- used in behaviors.json (attackerPassBallStrategy)

///////////////////
// SHOOT_BLOCKED //
///////////////////
bool isShortTurnToGoalBlockedByOpponent();      // <-- used in behaviors.json (lobShotOnGoal)
bool isLongTurnToGoalBlockedByOpponent();       // <-- used in behaviors.json (lobShotOnGoal)
//bool isShotOnGoalBlocked();      replaced by IsShootToPOIBlocked           // <-- used in behaviors.json (attackerPlayBall)

///////////
// RULES //
///////////
bool isValidNumberOfPassesGiven();          // <-- used in behaviors.json (attackerPlayBall)



///////////
// ????? //
///////////
bool isInScoringPosition();                 // <-- used in behaviors.json (attackerPlayBall)

//bool isAnAttackerOnOppHalf();  replaced by IsPOIInArea             // <-- (behaviors.json) defenderPassBallStrategy

#endif