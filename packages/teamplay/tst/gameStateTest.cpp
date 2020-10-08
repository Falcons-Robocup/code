 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameStateTest.cpp
 *
 *  Created on: February 16, 2017
 *      Author: Diana Koenraadt
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/types/gameState.hpp"

// **** Constructor tests *** //

// ================
// INVALID:
// When game state is invalid, play state can be anything. However, there can be no setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateInvalid_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    gameState g1(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::INVALID, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateInvalid_WhenOtherStatesInvalid_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF));
    EXPECT_ANY_THROW(gameState g2(governingGameState::INVALID, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF));
    EXPECT_ANY_THROW(gameState g3(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::GOALKICK));
    EXPECT_ANY_THROW(gameState g4(governingGameState::INVALID, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::GOALKICK));
}

// ================
// NEUTRAL_PLAYING:
// When game state is neutral playing:
// - can only be in play (out of play not allowed) 
// - there can be no setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenMatchStateInMatch_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenMatchStateOutOfMatch_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenSetpieceTypePenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralPlaying_WhenSetpieceOwnerOwn_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
}

// ================
// NEUTRAL_STOPPED:
// When game state is neutral stopped:
// - can be in or out of play
// - there can be no setpiece
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenMatchStateInMatch_WhenSetpieceTypeAndOwnerNone_ShouldNotThrow)
{
    // Arrange

    // Act
    gameState g1(governingGameState::NEUTRAL_STOPPED, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::NEUTRAL_STOPPED, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g3(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g4(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenSetpieceTypePenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState g3(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState g4(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateNeutralStopped_WhenSetpieceOwnerOwn_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState g1(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState g3(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState g4(governingGameState::NEUTRAL_PLAYING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::NONE));
}

// ================
// SETPIECE_PREPARING or SETPIECE_EXECUTING
// When game state is setpiece preparing:
// - can only be out of play when setpiece is PENALTY
// - can be in play for any setpiece
// - setpiece cannot be none
// - if setpiece is dropped ball then the owner must be none
// - If owner is none, setpiece must be dropped ball
// ================

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenMatchStateOutOfMatch_WhenSetpieceTypePenalty_ShouldNotThrow)
{
    // Arrange

    // Act
    gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);
    gameState gp2(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::PENALTY);
    gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);
    gameState ge2(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenMatchStateOutOfMatch_WhenSetpieceNotPenalty_ShouldThrow)
{
    // Arrange

    // Assert
    EXPECT_ANY_THROW(gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::GOALKICK));
    EXPECT_ANY_THROW(gameState gp2(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::GOALKICK));
    EXPECT_ANY_THROW(gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::GOALKICK));
    EXPECT_ANY_THROW(gameState ge2(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::GOALKICK));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetpieceNone_ShouldThrow)
{
    // Arrange

    // Assert
    // setpieceType::NONE with differing play states
    EXPECT_ANY_THROW(gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState gp2(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState gp3(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState gp4(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::NONE));

    // setpieceOwner::NONE with differing play states
    EXPECT_ANY_THROW(gameState gp5(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState gp6(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState gp7(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState gp8(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));

    // setpieceType::NONE with differing play states
    EXPECT_ANY_THROW(gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState ge2(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState ge3(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::NONE));
    EXPECT_ANY_THROW(gameState ge4(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::NONE));

    // setpieceOwner::NONE with differing play states
    EXPECT_ANY_THROW(gameState ge5(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState ge6(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState ge7(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
    EXPECT_ANY_THROW(gameState ge8(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNone_WhenSetpieceDroppedBall_ShouldNotThrow)
{
    // Arrange

    // Act
    gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL);

    gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL);

    // Assert
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNotNone_WhenSetpieceDroppedBall_ShouldThrow)
{
    // Arrange

    // Act

    // Assert
    EXPECT_ANY_THROW(gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::DROPPED_BALL));
    EXPECT_ANY_THROW(gameState gp2(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::DROPPED_BALL));

    EXPECT_ANY_THROW(gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::DROPPED_BALL));
    EXPECT_ANY_THROW(gameState ge2(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::DROPPED_BALL));
}

TEST(GameStateTest, constructor_WhenGoverningGameStateSetpiecePreparingExecuting_WhenSetPieceOwnerNone_WhenSetpieceNotDroppedBall_ShouldThrow)
{
    // Arrange

    // Act

    // Assert
    EXPECT_ANY_THROW(gameState gp1(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::KICKOFF));
    EXPECT_ANY_THROW(gameState gp2(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));

    EXPECT_ANY_THROW(gameState ge1(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::KICKOFF));
    EXPECT_ANY_THROW(gameState ge2(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::PENALTY));
}

// **** isInMatch tests *** //

// ================
// IN_PLAY should return true
// ================

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateInvalid_ShouldReturnTrue)
{
    // Arrange
    gameState g(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateNeutralPlaying_ShouldReturnTrue)
{
    // Arrange
    gameState g(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateNeutralStopped_ShouldReturnTrue)
{
    // Arrange
    gameState g(governingGameState::NEUTRAL_STOPPED, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateSetpiecePreparing_ShouldReturnTrue)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateInMatch_WhenGoverningGameStateSetpieceExecuting_ShouldReturnTrue)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL);

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
    gameState g(governingGameState::INVALID, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateNeutralStopped_ShouldReturnFalse)
{
    // Arrange
    gameState g(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpiecePreparingOwnPenalty_ShouldReturnFalse)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpiecePreparingOpponentPenalty_ShouldReturnFalse)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpieceExecutingOwnPenalty_ShouldReturnFalse)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, isInMatch_WhenMatchStateOutOfMatch_WhenGoverningGameStateSetpieceExecutingOpponentPenalty_ShouldReturnFalse)
{
    // Arrange
    gameState g(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    // Act
    bool result = g.isInMatch();

    // Assert
    EXPECT_FALSE(result);
}

// **** equality operator(==) tests *** //

TEST(GameStateTest, equality_allEqual_ShouldReturnTrue)
{
    // Arrange
    gameState g1(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_TRUE(result);
}

TEST(GameStateTest, equality_governingGameStateDifferent_ShouldReturnFalse)
{
    // Arrange
    gameState g1(governingGameState::INVALID, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_governingMatchStateDifferent_ShouldReturnFalse)
{
    // Arrange
    gameState g1(governingGameState::NEUTRAL_STOPPED, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::NONE);
    gameState g2(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_setpieceOwnerDifferent_ShouldReturnFalse)
{
    // Arrange
    gameState g1(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);
    gameState g2(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

TEST(GameStateTest, equality_setpieceTypeDifferent_ShouldReturnFalse)
{
    // Arrange
    gameState g1(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY);
    gameState g2(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF);

    // Act
    bool result = g1 == g2;

    // Assert
    EXPECT_FALSE(result);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
