 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameStateTransitionTableTest.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/gameStateTransitionTable.hpp"

class GameStateTransitionTableTest : public TeamplayTest
{
public:
    void expect_gamestate(const gameState currentGameState, const refboxSignalEnum refboxSignal, const gameState expectedGameState)
    {
        EXPECT_EQ(expectedGameState, gameStateTransitionTable::getInstance().calculateNewGameState(currentGameState, refboxSignal));
    }
};

TEST_F(GameStateTransitionTableTest, RefboxSignalStopInSeveralStates)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::STOP,
        gameState(governingGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::STOP,
        gameState(governingGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF),
        refboxSignalEnum::STOP,
        gameState(governingGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF),
        refboxSignalEnum::STOP,
        gameState(governingGameState::NEUTRAL_STOPPED));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnKickoff)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::KICKOFF_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::KICKOFF_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF),
        refboxSignalEnum::START,
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::KICKOFF));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOpponentKickoff)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::KICKOFF_OPP,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::KICKOFF));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::KICKOFF_OPP,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::KICKOFF));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::KICKOFF),
        refboxSignalEnum::START,
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::KICKOFF));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnThrowin)
{
    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::FREEKICK),
        refboxSignalEnum::THROWIN_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::THROWIN));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::THROWIN_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::THROWIN));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::THROWIN_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::THROWIN));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::THROWIN),
        refboxSignalEnum::START,
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::THROWIN));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOpponentCorner)
{
    expect_gamestate(
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL),
        refboxSignalEnum::CORNER_KICK_OPP,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::CORNER_KICK_OPP,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::CORNER_KICK_OPP,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER),
        refboxSignalEnum::START,
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsDroppedBall)
{
    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::GOALKICK),
        refboxSignalEnum::DROPPED_BALL,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::DROPPED_BALL,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::DROPPED_BALL,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL),
        refboxSignalEnum::START,
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::NONE, setpieceType::DROPPED_BALL));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnPenalty_WhenInMatch)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnPenalty_WhenOutOfMatch)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::PENALTY_KICK_OWN,
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::OUT_OF_MATCH, setpieceOwner::OWN, setpieceType::PENALTY));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsEndGame)
{
    expect_gamestate(
        gameState(governingGameState::NEUTRAL_STOPPED),
        refboxSignalEnum::END_GAME,
        gameState(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));

    expect_gamestate(
        gameState(governingGameState::NEUTRAL_PLAYING),
        refboxSignalEnum::END_GAME,
        gameState(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::END_GAME,
        gameState(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));

    expect_gamestate(
        gameState(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::PENALTY),
        refboxSignalEnum::END_GAME,
        gameState(governingGameState::NEUTRAL_STOPPED, governingMatchState::OUT_OF_MATCH, setpieceOwner::NONE, setpieceType::NONE));
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
