 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDecisionTreeTypes.hpp
 *
 *  Created on: Apr 24, 2016
 *      Author: Erik Kouters
 */

#ifndef CDECISIONTREETYPES_HPP_
#define CDECISIONTREETYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>

// sharedTypes
#include "treeEnum.hpp"
#include "behTreeReturnEnum.hpp"

static std::map<std::string, treeEnum> treeEnumMapping = boost::assign::map_list_of
    // GAMESTATES                                   // GAMESTATES
    ("neutralStopped_neutral",                      treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL)
    ("neutralPlaying_neutral",                      treeEnum::IN_MATCH_NEUTRAL_PLAYING_NEUTRAL)
    ("ownKickoffPrepare_neutral",                   treeEnum::IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL)
    ("oppKickoffPrepare_neutral",                   treeEnum::IN_MATCH_OPP_KICKOFF_PREPARE_NEUTRAL)
    ("ownKickoffExecute_neutral",                   treeEnum::IN_MATCH_OWN_KICKOFF_EXECUTE_NEUTRAL)
    ("oppKickoffExecute_neutral",                   treeEnum::IN_MATCH_OPP_KICKOFF_EXECUTE_NEUTRAL)
    ("ownFreekickPrepare_neutral",                  treeEnum::IN_MATCH_OWN_FREEKICK_PREPARE_NEUTRAL)
    ("oppFreekickPrepare_neutral",                  treeEnum::IN_MATCH_OPP_FREEKICK_PREPARE_NEUTRAL)
    ("ownFreekickExecute_neutral",                  treeEnum::IN_MATCH_OWN_FREEKICK_EXECUTE_NEUTRAL)
    ("oppFreekickExecute_neutral",                  treeEnum::IN_MATCH_OPP_FREEKICK_EXECUTE_NEUTRAL)
    ("ownGoalkickPrepare_neutral",                  treeEnum::IN_MATCH_OWN_GOALKICK_PREPARE_NEUTRAL)
    ("oppGoalkickPrepare_neutral",                  treeEnum::IN_MATCH_OPP_GOALKICK_PREPARE_NEUTRAL)
    ("ownGoalkickExecute_neutral",                  treeEnum::IN_MATCH_OWN_GOALKICK_EXECUTE_NEUTRAL)
    ("oppGoalkickExecute_neutral",                  treeEnum::IN_MATCH_OPP_GOALKICK_EXECUTE_NEUTRAL)
    ("ownThrowinPrepare_neutral",                   treeEnum::IN_MATCH_OWN_THROWIN_PREPARE_NEUTRAL)
    ("oppThrowinPrepare_neutral",                   treeEnum::IN_MATCH_OPP_THROWIN_PREPARE_NEUTRAL)
    ("ownThrowinExecute_neutral",                   treeEnum::IN_MATCH_OWN_THROWIN_EXECUTE_NEUTRAL)
    ("oppThrowinExecute_neutral",                   treeEnum::IN_MATCH_OPP_THROWIN_EXECUTE_NEUTRAL)
    ("ownCornerPrepare_neutral",                    treeEnum::IN_MATCH_OWN_CORNER_PREPARE_NEUTRAL)
    ("oppCornerPrepare_neutral",                    treeEnum::IN_MATCH_OPP_CORNER_PREPARE_NEUTRAL)
    ("ownCornerExecute_neutral",                    treeEnum::IN_MATCH_OWN_CORNER_EXECUTE_NEUTRAL)
    ("oppCornerExecute_neutral",                    treeEnum::IN_MATCH_OPP_CORNER_EXECUTE_NEUTRAL)
    ("ownPenaltyPrepare_neutral",                   treeEnum::IN_MATCH_OWN_PENALTY_PREPARE_NEUTRAL)
    ("oppPenaltyPrepare_neutral",                   treeEnum::IN_MATCH_OPP_PENALTY_PREPARE_NEUTRAL)
    ("ownPenaltyExecute_neutral",                   treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL)
    ("oppPenaltyExecute_neutral",                   treeEnum::IN_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL)
    ("droppedBallPrepare_neutral",                  treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL)
    ("droppedBallExecute_neutral",                  treeEnum::IN_MATCH_DROPPED_BALL_EXECUTE_NEUTRAL)

    ("outOfMatchNeutralStopped",                    treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL)
    ("ownShootoutPrepare",                          treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL) // An out of match penalty is called a shootout
    ("oppShootoutPrepare",                          treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL)
    ("ownShootoutExecute",                          treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL)
    ("oppShootoutExecute",                          treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL)
    ("parking",                                     treeEnum::PARKING)

    // Roles                                        // ROLES
    ("attackerMain",                                treeEnum::ATTACKER_MAIN)
    ("attackerAssist",                              treeEnum::ATTACKER_ASSIST)
    ("defenderMain",                                treeEnum::DEFENDER_MAIN)
    ("defenderAssist",                              treeEnum::DEFENDER_ASSIST)
    ("R_goalkeeper",                                treeEnum::R_GOALKEEPER)
    ("R_robotStop",                                 treeEnum::R_ROBOT_STOP)

    // Behaviors                                    // BEHAVIORS
    ("B_goalkeeper",                                treeEnum::B_GOALKEEPER)
    ("B_robotStop",                                 treeEnum::B_ROBOT_STOP)
    ("attackerPassBallStrategy",                    treeEnum::ATTACKER_PASS_BALL_STRATEGY)
    ("attackerPlayBall",                            treeEnum::ATTACKER_PLAY_BALL)
    ("attackMain",                                  treeEnum::ATTACK_MAIN)
    ("attackAssist",                                treeEnum::ATTACK_ASSIST)
    ("defendAssist",                                treeEnum::DEFEND_ASSIST)
    ("defendPotentialOppAttacker",                  treeEnum::DEFEND_POTENTIAL_OPP_ATTACKER)
    ("defendMain",                                  treeEnum::DEFEND_MAIN)
    ("defenderPassBallStrategy",                    treeEnum::DEFENDER_PASS_BALL_STRATEGY)
    ("defenderPlayBall",                            treeEnum::DEFENDER_PLAY_BALL)
    ("dropBallExecute",                             treeEnum::DROPBALL_EXECUTE)
    ("dropBallPrepare",                             treeEnum::DROPBALL_PREPARE)
    ("dropBallSearch",                                 treeEnum::DROPBALL_SEARCH)
    ("getBall",                                     treeEnum::GET_BALL)
    ("oppCornerPrepare",                            treeEnum::OPP_CORNER_PREPARE)
    ("oppCornerSearch",                               treeEnum::OPP_CORNER_SEARCH)
    ("oppFreekickPrepare",                          treeEnum::OPP_FREEKICK_PREPARE)
    ("oppFreekickSearch",                              treeEnum::OPP_FREEKICK_SEARCH)
    ("oppGoalkickPrepare",                          treeEnum::OPP_GOALKICK_PREPARE)
    ("oppGoalkickSearch",                              treeEnum::OPP_GOALKICK_SEARCH)
    ("oppKickoffPrepare",                           treeEnum::OPP_KICKOFF_PREPARE)
    ("oppKickoffSearch",                               treeEnum::OPP_KICKOFF_SEARCH)
    ("oppPenaltyPrepare",                              treeEnum::OPP_PENALTY_PREPARE)
    ("oppThrowinPrepare",                           treeEnum::OPP_THROWIN_PREPARE)
    ("oppThrowinSearch",                               treeEnum::OPP_THROWIN_SEARCH)
    ("ownCornerExecute",                            treeEnum::OWN_CORNER_EXECUTE)
    ("ownCornerPrepare",                            treeEnum::OWN_CORNER_PREPARE)
    ("ownCornerSearch",                                treeEnum::OWN_CORNER_SEARCH)
    ("ownFreekickExecute",                          treeEnum::OWN_FREEKICK_EXECUTE)
    ("ownFreekickPrepare",                          treeEnum::OWN_FREEKICK_PREPARE)
    ("ownFreekickSearch",                              treeEnum::OWN_FREEKICK_SEARCH)
    ("ownGoalkickExecute",                          treeEnum::OWN_GOALKICK_EXECUTE)
    ("ownGoalkickPrepare",                          treeEnum::OWN_GOALKICK_PREPARE)
    ("ownGoalkickSearch",                              treeEnum::OWN_GOALKICK_SEARCH)
    ("ownKickoffExecute",                           treeEnum::OWN_KICKOFF_EXECUTE)
    ("ownKickoffPrepare",                           treeEnum::OWN_KICKOFF_PREPARE)
    ("ownKickoffSearch",                               treeEnum::OWN_KICKOFF_SEARCH)
    ("ownPenaltyExecute",                           treeEnum::OWN_PENALTY_EXECUTE)
    ("ownPenaltyPrepare",                              treeEnum::OWN_PENALTY_PREPARE)
    ("ownThrowinExecute",                           treeEnum::OWN_THROWIN_EXECUTE)
    ("ownThrowinPrepare",                           treeEnum::OWN_THROWIN_PREPARE)
    ("ownThrowinSearch",                               treeEnum::OWN_THROWIN_SEARCH)
    ("passBallToClosestAttacker",                   treeEnum::PASS_BALL_TO_CLOSEST_ATTACKER)
    ("passBallToClosestAttackerOnOppHalf",          treeEnum::PASS_BALL_TO_CLOSEST_ATTACKER_ON_OPP_HALF)
    ("passBallToFurthestAttacker",                  treeEnum::PASS_BALL_TO_FURTHEST_ATTACKER)
    ("passBallToClosestTeammember",                 treeEnum::PASS_BALL_TO_CLOSEST_TEAMMEMBER)
    ("passBallToFurthestDefender",                  treeEnum::PASS_BALL_TO_FURTHEST_DEFENDER)
    ("positionToFreeSpot",                          treeEnum::POSITION_TO_FREE_SPOT)
    ("receivePass",                                       treeEnum::RECEIVE_PASS)
    ("searchBall",                                  treeEnum::SEARCH_BALL)
    ("setPiece",                                      treeEnum::SETPIECE)
    ("setPieceSearchBall",                            treeEnum::SETPIECE_SEARCH_BALL)
    ("shootAtGoal",                                 treeEnum::SHOOT_AT_GOAL)
    ("lobShotOnGoal",                               treeEnum::LOB_SHOT_ON_GOAL)
    ("tipInAssist",                                 treeEnum::TIP_IN_ASSIST)
    ("tipInExecute",                                 treeEnum::TIP_IN_EXECUTE)
    ("turnAwayFromClosestOpponent",                 treeEnum::TURN_AWAY_FROM_CLOSEST_OPPONENT)
    ("testObstacleStrafe",                          treeEnum::TEST_OBSTACLE_STRAFE)
    ("park",                                        treeEnum::PARK)
    ;

// Implemented in cDecisionTree.cpp
treeEnum treeStrToEnum(const std::string enumString);
// Implemented in cDecisionTree.cpp
std::string treeEnumToStr(const treeEnum treeEnumVal);

enum class nodeEnum
{
    INVALID = 0,

    WORLDSTATE_FUNCTION,
    MEMWORLDSTATE_FUNCTION,
    RULE,
    ROLE,
    BEHAVIOR,
    ACTION,
    SEQUENCE,
    MEMSEQUENCE,
    SELECTOR,
    MEMSELECTOR,
    PARAMETER_CHECK,
    REPEATER,
    INVERTER
};


static std::map<std::string, nodeEnum> nodeTypeMapping = boost::assign::map_list_of
        ("invalid", nodeEnum::INVALID)
        ("worldStateFunction", nodeEnum::WORLDSTATE_FUNCTION)
        ("memWorldStateFunction", nodeEnum::MEMWORLDSTATE_FUNCTION)
        ("rule", nodeEnum::RULE)
        ("role", nodeEnum::ROLE)
        ("behavior", nodeEnum::BEHAVIOR)
        ("action", nodeEnum::ACTION)
        ("sequence", nodeEnum::SEQUENCE)
        ("memsequence", nodeEnum::MEMSEQUENCE)
        ("selector", nodeEnum::SELECTOR)
        ("memselector", nodeEnum::MEMSELECTOR)
        ("parameterCheck", nodeEnum::PARAMETER_CHECK)
        ;

static std::map<std::string, behTreeReturnEnum> behTreeReturnMapping = boost::assign::map_list_of
        ("INVALID", behTreeReturnEnum::INVALID)
        ("PASSED", behTreeReturnEnum::PASSED)
        ("FAILED", behTreeReturnEnum::FAILED)
        ("RUNNING", behTreeReturnEnum::RUNNING)
        ;

static std::map<behTreeReturnEnum, std::string> behTreeReturnReverseMapping = boost::assign::map_list_of
        (behTreeReturnEnum::INVALID, "INVALID")
        (behTreeReturnEnum::PASSED, "PASSED")
        (behTreeReturnEnum::FAILED, "FAILED")
        (behTreeReturnEnum::RUNNING, "RUNNING")
        ;

#endif /* CDECISIONTREETYPES_HPP_ */
