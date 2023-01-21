// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Bring in gtest
#include "gtest/gtest.h"

/* Include trace utility */
#include "tracing.hpp"

// Bring in my package's API, which is what I'm testing
#include "int/controlnodes/AdvancedQueries.hpp"
#include "int/stores/BallStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/GameStateStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "int/types/BehaviorTreeTypes.hpp"

#include <math.h>

using namespace ::testing;
using namespace teamplay;


// ======================
// IsShootToPOIBlocked
// ======================

TEST(TestSuiteWorldStateFunctionsisPassToClosestTeammemberBlocked, Teammember1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Member2 @0,5
    // Bot1 @0,2
    // Radius ball = 0.125

    std::vector<Obstacle> obstacles;

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(0.0, 5.0, 0.0), geometry::Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 2.0, 0.0)));

    // Execution
    auto poi = BT::convertFromString<Point2D>("closestTeammember");
    EXPECT_TRUE(isShootToPOIBlocked(poi, tpShootTypeEnum::PASS));
}

TEST(TestSuiteWorldStateFunctionsisPassToClosestTeammemberBlocked, Teammember2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Member2 @0,5
    // Bot1 @0,7
    // Radius ball = 0.125

    std::vector<Obstacle> obstacles;

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(0.0, 5.0, 0.0), geometry::Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 7.0, 0.0)));

    // Execution
    auto poi = BT::convertFromString<Point2D>("closestTeammember");
    EXPECT_FALSE(isShootToPOIBlocked(poi, tpShootTypeEnum::PASS));
}

TEST(TestSuiteWorldStateFunctionsisPassToDefenderClosestToOpponentGoalBlocked, test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,2
    // Member1 (DEFENDER_ASSIST) @0,0
    // Member2 (DEFENDER_MAIN) @0,-2
    // Add obstacle @0,1
    // Radius ball = 0.125

    std::vector<Obstacle> obstacles;

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 2.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(4.0, -2.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(3, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 1.0, 0.0)));

    // Execution
    auto poi = BT::convertFromString<Point2D>("defenderClosestToOpponentGoal");
    EXPECT_TRUE(isShootToPOIBlocked(poi, tpShootTypeEnum::PASS));
}

TEST(TestSuiteWorldStateFunctionsisPassToDefenderClosestToOpponentGoalBlocked, test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,2
    // Member1 (DEFENDER_ASSIST) @0,0
    // Member2 (DEFENDER_MAIN) @0,-2
    // Add obstacle @4,1
    // Radius ball = 0.125

    std::vector<Obstacle> obstacles;

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(4.0, 2.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(4.0, -2.0, 0.0), geometry::Velocity2D()));
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(3, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(4.0, 0.0, 0.0)));

    // Execution
    auto poi = BT::convertFromString<Point2D>("defenderClosestToOpponentGoal");
    EXPECT_FALSE(isShootToPOIBlocked(poi, tpShootTypeEnum::PASS));
}

TEST(TestSuiteWorldStateFunctionsisLobShotOnGoalBlocked, Opponent1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,1
    // Radius ball = 0.125

    std::vector<Obstacle> obstacles;

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D()));

    // Set teammember on Target
    std::vector<RobotNumber> activeRobots;
    std::vector<Robot> teammembers;

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 1.0, 0.0)));

    // Execution
    // TODO -- lobshot logic is different (blocked when distance to obstacle < 0.6 + ROBOT_RADIUS)
    //EXPECT_FALSE(isLobShotOnGoalBlocked());
    auto poi = BT::convertFromString<Point2D>("P_OPP_GOALLINE_CENTER");
    EXPECT_FALSE(isShootToPOIBlocked(poi, tpShootTypeEnum::LOB_SHOT));
}





// ======================
// isSetPiece
// ======================

class GameStateQueriesTest : public Test { };

TEST_F(GameStateQueriesTest, InvalidGameState)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::INVALID);

    // Do not test playstate, it doesn't matter when the state is invalid.
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, NeutralStopped)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::NEUTRAL_STOPPED);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, NeutralPlaying)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::NEUTRAL_PLAYING);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, DroppedBallPrepare)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_PREPARING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::NONE,
                                                  SetpieceType::DROPPED_BALL);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, DroppedBallExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::NONE,
                                                  SetpieceType::DROPPED_BALL);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnKickoffPrepare)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_PREPARING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::KICKOFF);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnKickoffExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::KICKOFF);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OpponentKickoffPrepare)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_PREPARING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::KICKOFF);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OpponentKickoffExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::KICKOFF);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnFreekickExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::FREEKICK);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnGoalkickExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::GOALKICK);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnThrowinExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::THROWIN);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnCornerExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::CORNER);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OwnPenaltyExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::PENALTY);

    EXPECT_TRUE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OutOfMatchNeutralStopped)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::NEUTRAL_STOPPED,
                                                  GoverningMatchState::OUT_OF_MATCH,
                                                  SetpieceOwner::NONE,
                                                  SetpieceType::NONE);

    EXPECT_FALSE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyPrepare)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_PREPARING,
                                                  GoverningMatchState::OUT_OF_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::PENALTY);

    EXPECT_FALSE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::OUT_OF_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::PENALTY);

    EXPECT_FALSE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyPrepare)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_PREPARING,
                                                  GoverningMatchState::OUT_OF_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::PENALTY);

    EXPECT_FALSE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyExecute)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::OUT_OF_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::PENALTY);

    EXPECT_FALSE(GameStateStore::getInstance().getGameState().isInMatch());
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::ANY));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::OWN));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::PREPARE));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::DROPPED_BALL));
    EXPECT_FALSE(isSetpiece(SetpieceTreeTypeEnum::KICKOFF));
    EXPECT_TRUE(isSetpiece(SetpieceTreeTypeEnum::PENALTY));
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
