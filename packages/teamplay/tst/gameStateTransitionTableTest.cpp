// Copyright 2016-2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
