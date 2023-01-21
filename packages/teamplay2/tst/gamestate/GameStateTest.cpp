// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateTest.cpp
 *
 *  Created on: February 16, 2017
 *      Author: Diana Koenraadt
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/types/GameState.hpp"

// **** Constructor tests *** //

// ================
// INVALID:
// When game state is invalid, play state can be anything. However, there can be no Setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateInvalid_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    GameState g1(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::INVALID, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateInvalid_WhenOtherStatesInvalid_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::INVALID, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF));
    EXPECT_ANY_THROW(GameState g3(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::GOALKICK));
    EXPECT_ANY_THROW(GameState g4(GoverningGameState::INVALID, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::GOALKICK));
}

// ================
// NEUTRAL_PLAYING:
// When game state is neutral playing:
// - can only be in play (out of play not allowed) 
// - there can be no Setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenMatchStateInMatch_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenMatchStateOutOfMatch_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenSetpieceTypePenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenSetpieceOwnerOwn_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
}

// ================
// NEUTRAL_STOPPED:
// When game state is neutral stopped:
// - can be in or out of play
// - there can be no Setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenMatchStateInMatch_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    GameState g1(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g3(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g4(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenSetpieceTypePenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState g3(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState g4(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenSetpieceOwnerOwn_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState g1(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState g3(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState g4(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
}

// ================
// SETPIECE_PREPARING or SETPIECE_EXECUTING
// When game state is Setpiece preparing:
// - can only be out of play when Setpiece is PENALTY
// - can be in play for any Setpiece
// - Setpiece cannot be none
// - if Setpiece is dropped ball then the owner must be none
// - If owner is none, Setpiece must be dropped ball
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenMatchStateOutOfMatch_WhenSetpieceTypePenalty_ShouldNotThrow)
{
    // Arrange

    // Act
    GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);
    GameState gp2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::PENALTY);
    GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);
    GameState ge2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::PENALTY);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenMatchStateOutOfMatch_WhenSetpieceNotPenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::GOALKICK));
    EXPECT_ANY_THROW(GameState gp2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::GOALKICK));
    EXPECT_ANY_THROW(GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::GOALKICK));
    EXPECT_ANY_THROW(GameState ge2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::GOALKICK));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetpieceNone_ShouldThrow)
{
    // Arrange

    // Assert
    // SetpieceType::NONE with differing play states
    EXPECT_ANY_THROW(GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState gp2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState gp3(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState gp4(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::NONE));

    // SetpieceOwner::NONE with differing play states
    EXPECT_ANY_THROW(GameState gp5(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState gp6(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState gp7(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState gp8(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));

    // SetpieceType::NONE with differing play states
    EXPECT_ANY_THROW(GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState ge2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState ge3(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::NONE));
    EXPECT_ANY_THROW(GameState ge4(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::NONE));

    // SetpieceOwner::NONE with differing play states
    EXPECT_ANY_THROW(GameState ge5(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState ge6(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState ge7(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
    EXPECT_ANY_THROW(GameState ge8(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNone_WhenSetpieceDroppedBall_ShouldNotThrow)
{
    // Arrange

    // Act
    GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL);

    GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNotNone_WhenSetpieceDroppedBall_ShouldThrow)
{
    // Arrange

    // Act

    // Assert
    EXPECT_ANY_THROW(GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::DROPPED_BALL));
    EXPECT_ANY_THROW(GameState gp2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::DROPPED_BALL));

    EXPECT_ANY_THROW(GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::DROPPED_BALL));
    EXPECT_ANY_THROW(GameState ge2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::DROPPED_BALL));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNone_WhenSetpieceNotDroppedBall_ShouldThrow)
{
    // Arrange

    // Act

    // Assert
    EXPECT_ANY_THROW(GameState gp1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::KICKOFF));
    EXPECT_ANY_THROW(GameState gp2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));

    EXPECT_ANY_THROW(GameState ge1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::KICKOFF));
    EXPECT_ANY_THROW(GameState ge2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::PENALTY));
}

// **** isInMatch tests *** //

// ================
// IN_PLAY should return true
// ================

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateInvalid_ShouldReturnTrue)
{
    // Arrange
    GameState g(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateNeutralPlaying_ShouldReturnTrue)
{
    // Arrange
    GameState g(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateNeutralStopped_ShouldReturnTrue)
{
    // Arrange
    GameState g(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateSetpiecePreparing_ShouldReturnTrue)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateSetpieceExecuting_ShouldReturnTrue)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

// ================
// OUT_OF_PLAY should return false
// ================

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateInvalid_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::INVALID, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateNeutralStopped_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpiecePreparingOwnPenalty_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpiecePreparingOpponentPenalty_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpieceExecutingOwnPenalty_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpieceExecutingOpponentPenalty_ShouldReturnFalse)
{
    // Arrange
    GameState g(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

// **** equality operator(==) tests *** //

TEST(GameStateTest, equality_allEqual_ShouldReturnTrue)
{
    // Arrange
    GameState g1(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, equality_GoverningGameStateDifferent_ShouldReturnFalse)
{
    // Arrange
    GameState g1(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_GoverningMatchStateDifferent_ShouldReturnFalse)
{
    // Arrange
    GameState g1(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
    GameState g2(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_SetpieceOwnerDifferent_ShouldReturnFalse)
{
    // Arrange
    GameState g1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);
    GameState g2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OPPONENT, SetpieceType::PENALTY);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_SetpieceTypeDifferent_ShouldReturnFalse)
{
    // Arrange
    GameState g1(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY);
    GameState g2(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, CorrectlyStoresNeutralPlaying)
{
    GameState g(GoverningGameState::NEUTRAL_PLAYING);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_FALSE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
}

TEST(GameStateTest, CorrectlyStoresAValidSetpiece)
{
    GameState g(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_TRUE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
}

TEST(GameStateTest, DoesNotStoreASetpieceWithoutAttributes)
{
    EXPECT_ANY_THROW(GameState g(GoverningGameState::SETPIECE_PREPARING));
}

TEST(GameStateTest, DoesNotStoreNeutralPlayingWithSetpieceAttributes)
{
    EXPECT_ANY_THROW(GameState g(GoverningGameState::NEUTRAL_PLAYING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER));
}

TEST(GameStateTest, EqualityTest)
{
    GameState g1(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::FREEKICK);
    GameState g2(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::FREEKICK);
    GameState g3(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::GOALKICK);
    GameState g4(GoverningGameState::NEUTRAL_PLAYING);

    EXPECT_TRUE(g1 == g2);
    EXPECT_FALSE(g1 != g2);

    EXPECT_FALSE(g2 == g3);
    EXPECT_TRUE(g2 != g3);

    EXPECT_FALSE(g3 == g4);
    EXPECT_TRUE(g3 != g4);

    EXPECT_TRUE(g4 == g4);
    EXPECT_FALSE(g4 != g4);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
