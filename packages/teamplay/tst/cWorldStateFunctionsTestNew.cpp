 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldStateFunctionsTestNew.cpp
 * New WSF tester: will eventually replace the existing WSF tester. Work in progress (by Coen)
 *
 *  Created on: Jul 24, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "gtest/gtest.h"

/* Include trace utility */
#include "int/utilities/trace.hpp"

/* SUT */
#include "int/stores/ballStore.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/ownRobotStore.hpp"
#include "int/stores/teamMatesStore.hpp"
#include "int/types/robot.hpp"
#include "int/cWorldStateFunctions.hpp"


using namespace ::testing;
using namespace teamplay;

class GameStateTest : public Test { };

TEST_F(GameStateTest, CorrectlyStoresNeutralPlaying)
{
    gameState g(governingGameState::NEUTRAL_PLAYING);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_FALSE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
    EXPECT_FALSE(g.isSidelineSetPiece());
}

TEST_F(GameStateTest, CorrectlyStoresAValidSetpiece)
{
    gameState g(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_TRUE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
    EXPECT_TRUE(g.isSidelineSetPiece());
}

TEST_F(GameStateTest, DoesNotStoreASetpieceWithoutAttributes)
{
    EXPECT_ANY_THROW(gameState g(governingGameState::SETPIECE_PREPARING));
}

TEST_F(GameStateTest, DoesNotStoreNeutralPlayingWithSetpieceAttributes)
{
    EXPECT_ANY_THROW(gameState g(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));
}

TEST_F(GameStateTest, EqualityTest)
{
    gameState g1(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::FREEKICK);
    gameState g2(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::FREEKICK);
    gameState g3(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::GOALKICK);
    gameState g4(governingGameState::NEUTRAL_PLAYING);

    EXPECT_TRUE(g1 == g2);
    EXPECT_FALSE(g1 != g2);

    EXPECT_FALSE(g2 == g3);
    EXPECT_TRUE(g2 != g3);

    EXPECT_FALSE(g3 == g4);
    EXPECT_TRUE(g3 != g4);

    EXPECT_TRUE(g4 == g4);
    EXPECT_FALSE(g4 != g4);
}


class GameStateQueriesTest : public Test { };

TEST_F(GameStateQueriesTest, InvalidGameState)
{
    gameStateStore::getInstance().updateGameState(governingGameState::INVALID);
    EXPECT_EQ(treeEnum::INVALID, gameStateStore::getInstance().getGameState_treeEnum());

    // Do not test playstate, it doesn't matter when the state is invalid.
    EXPECT_FALSE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, NeutralStopped)
{
    gameStateStore::getInstance().updateGameState(governingGameState::NEUTRAL_STOPPED);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_FALSE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, NeutralStopped_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_FALSE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, NeutralPlaying)
{
    gameStateStore::getInstance().updateGameState(governingGameState::NEUTRAL_PLAYING);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_PLAYING_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_FALSE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, DroppedBallPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::NONE, setpieceType::DROPPED_BALL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_TRUE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, DroppedBallPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_TRUE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, DroppedBallExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::NONE, setpieceType::DROPPED_BALL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_TRUE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnKickoffPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::OWN, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_TRUE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnKickoffExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_KICKOFF_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_TRUE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OpponentKickoffPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OPP_KICKOFF_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_TRUE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OpponentKickoffExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OPP_KICKOFF_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_TRUE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnFreekickExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::FREEKICK);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_FREEKICK_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnGoalkickExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::GOALKICK);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_GOALKICK_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnThrowinExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::THROWIN);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_THROWIN_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_TRUE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnCornerExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_CORNER_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_TRUE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnPenaltyExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::PENALTY);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OwnPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_TRUE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OutOfMatchNeutralStopped_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_FALSE(isInMatch());
    EXPECT_FALSE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_FALSE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_FALSE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_FALSE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_TRUE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_FALSE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_TRUE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    EXPECT_FALSE(isInMatch());
    EXPECT_TRUE(isSetPiece());
    EXPECT_FALSE(isOwnSetPiece());
    EXPECT_FALSE(isPrepareSetPiece());
    EXPECT_FALSE(isDroppedBallSetPiece());
    EXPECT_FALSE(isKickoffSetPiece());
    EXPECT_TRUE(isPenaltySetPiece());
    EXPECT_FALSE(isSidelineSetPiece());
}


class BallQueriesTest : public Test { };

TEST_F(BallQueriesTest, ballAtLeftSideOfField)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    ballStore::getBall().setPosition(Point3D(-1.0, 0.0, 0.0));
    EXPECT_FALSE(isSidelineSetPieceRight());
}

TEST_F(BallQueriesTest, lastKnownBallAtLeftSideOfField)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    ballStore::getBall().setPositionUnknown(Point3D(-1.0, 0.0, 0.0));
    EXPECT_FALSE(isSidelineSetPieceRight());
}

TEST_F(BallQueriesTest, ballAtRightSideOfField)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    ballStore::getBall().setPosition(Point3D(1.0, 0.0, 0.0));
    EXPECT_TRUE(isSidelineSetPieceRight());
}

TEST_F(BallQueriesTest, lastKnownBallAtRightSideOfField)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    ballStore::getBall().setPositionUnknown(Point3D(1.0, 0.0, 0.0));
    EXPECT_TRUE(isSidelineSetPieceRight());
}


class TeamTest : public Test
{
public:
    TeamTest()
    {
        teamMatesStore::getTeamMatesIncludingGoalie().clear();

        teamMatesStore::getTeamMatesIncludingGoalie().add(robot(1, treeEnum::R_GOALKEEPER, Position2D(), Velocity2D()));
        teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_MAIN, Position2D(), Velocity2D()));
        teamMatesStore::getTeamMatesIncludingGoalie().add(robot(3, treeEnum::ATTACKER_MAIN, Position2D(), Velocity2D()));
        teamMatesStore::getTeamMatesIncludingGoalie().add(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(), Velocity2D()));
        teamMatesStore::getTeamMatesIncludingGoalie().add(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(), Velocity2D()));
    }
};

TEST_F(TeamTest, ownRobotIsLowestActiveRobot)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(1);
    ownRobotStore::getOwnRobot().setNumber(1);

    EXPECT_TRUE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());
}

TEST_F(TeamTest, ownRobotIsHighestActiveRobot)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(5);
    ownRobotStore::getOwnRobot().setNumber(5);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_TRUE(isHighestActiveRobotID());
}

TEST_F(TeamTest, ownRobotIsSecondHighestActiveRobot)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(4);
    ownRobotStore::getOwnRobot().setNumber(4);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_TRUE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());
}

TEST_F(TeamTest, ownRobotIsThirdHighestActiveRobot)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(3);
    ownRobotStore::getOwnRobot().setNumber(3);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_TRUE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());
}

TEST_F(TeamTest, ownRobotIsOnlyActiveRobot)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(1);
    ownRobotStore::getOwnRobot().setNumber(1);
    EXPECT_FALSE(isOnlyActiveRobotID());

    teamMatesStore::getTeamMatesIncludingGoalie().clear();
    EXPECT_TRUE(isOnlyActiveRobotID());
}

TEST_F(TeamTest, fieldAssistentsAreNotPresent)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(4);
    teamMatesStore::getTeamMatesIncludingGoalie().remove(5);

    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_MAIN));
    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_MAIN));
}

TEST_F(TeamTest, defenderAssistentIsPresent)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(5);

    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_MAIN));

    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_MAIN));
    EXPECT_EQ( treeEnum::DEFENDER_ASSIST, teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_MAIN)->getRole() );

    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_ASSIST));
    EXPECT_EQ( treeEnum::DEFENDER_MAIN, teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_ASSIST)->getRole() );
}

TEST_F(TeamTest, attackerAssistentIsPresent)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(4);

    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_MAIN));

    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_MAIN));
    EXPECT_EQ( treeEnum::ATTACKER_ASSIST, teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_MAIN)->getRole() );

    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_ASSIST));
    EXPECT_EQ( treeEnum::ATTACKER_MAIN, teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_ASSIST)->getRole() );
}

TEST_F(TeamTest, bothAssistentsArePresent)
{
    EXPECT_FALSE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_MAIN));
    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::DEFENDER_ASSIST));
    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_MAIN));
    EXPECT_TRUE(teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(treeEnum::ATTACKER_ASSIST));
}

TEST_F(TeamTest, allRobotsAreActive)
{
    EXPECT_EQ(5, teamMatesStore::getTeamMatesIncludingGoalie().getNumberOfRobots());
}

TEST_F(TeamTest, notAllRobotsAreActive)
{
    teamMatesStore::getTeamMatesIncludingGoalie().remove(4);
    EXPECT_EQ(4, teamMatesStore::getTeamMatesIncludingGoalie().getNumberOfRobots());
}


class PositionsTest : public Test
{
public:
    PositionsTest()
    {
        robots.push_back(robot(1, treeEnum::R_GOALKEEPER, Position2D(-3.0, -4.1, 0.0), Velocity2D()));
        robots.push_back(robot(2, treeEnum::DEFENDER_MAIN, Position2D(3.0, -4.2, 0.0), Velocity2D()));
        robots.push_back(robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.5, 0.0), Velocity2D()));
        robots.push_back(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(-2.5, 1.5, 0.0), Velocity2D()));
        robots.push_back(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(2.0, 1.0, 0.0), Velocity2D()));
    }

    teamplay::robots robots;
};

TEST_F(PositionsTest, getRobotClosestToPoint)
{
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-3.0, -4.1)).getNumber());
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-2.0, -3.0)).getNumber());
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-3.0, -10.0)).getNumber());
    EXPECT_EQ(2, getRobotClosestToPoint(robots, Point2D(2.0, -2.0)).getNumber());
    EXPECT_EQ(3, getRobotClosestToPoint(robots, Point2D(0.0, 0.0)).getNumber());
    EXPECT_EQ(4, getRobotClosestToPoint(robots, Point2D(-2.0, 1.0)).getNumber());
    EXPECT_EQ(5, getRobotClosestToPoint(robots, Point2D(1.5, 0.5)).getNumber());
}

TEST_F(PositionsTest, throwsWhenNoRobotsGiven)
{
    robots.clear();
    EXPECT_ANY_THROW(getRobotClosestToPoint(robots, Point2D(0.0, 0.0)));
}


int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
