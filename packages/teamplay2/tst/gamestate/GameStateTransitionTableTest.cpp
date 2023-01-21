// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateTransitionTableTest.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/gamestate/GameStateTransitionTable.hpp"

class GameStateTransitionTableTest : public TeamplayTest
{
public:
    void expect_gamestate(const GameState currentGameState, const RefboxSignalEnum RefboxSignal, const GameState expectedGameState)
    {
        EXPECT_EQ(expectedGameState, GameStateTransitionTable::getInstance().calculateNewGameState(currentGameState, RefboxSignal));
    }
};

TEST_F(GameStateTransitionTableTest, RefboxSignalStopInSeveralStates)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::STOP,
        GameState(GoverningGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::STOP,
        GameState(GoverningGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF),
        RefboxSignalEnum::STOP,
        GameState(GoverningGameState::NEUTRAL_STOPPED));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF),
        RefboxSignalEnum::STOP,
        GameState(GoverningGameState::NEUTRAL_STOPPED));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnKickoff)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::KICKOFF_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::KICKOFF_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF),
        RefboxSignalEnum::START,
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::KICKOFF));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOpponentKickoff)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::KICKOFF_OPP,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::KICKOFF));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::KICKOFF_OPP,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::KICKOFF));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::KICKOFF),
        RefboxSignalEnum::START,
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::KICKOFF));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnThrowin)
{
    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::FREEKICK),
        RefboxSignalEnum::THROWIN_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::THROWIN));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::THROWIN_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::THROWIN));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::THROWIN_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::THROWIN));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::THROWIN),
        RefboxSignalEnum::START,
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::THROWIN));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOpponentCorner)
{
    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL),
        RefboxSignalEnum::CORNER_KICK_OPP,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::CORNER_KICK_OPP,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::CORNER_KICK_OPP,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER),
        RefboxSignalEnum::START,
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OPPONENT, SetpieceType::CORNER));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsDroppedBall)
{
    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::GOALKICK),
        RefboxSignalEnum::DROPPED_BALL,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::DROPPED_BALL,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::DROPPED_BALL,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL),
        RefboxSignalEnum::START,
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::DROPPED_BALL));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnPenalty_WhenInMatch)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsOwnPenalty_WhenOutOfMatch)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::PENALTY_KICK_OWN,
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY));
}

TEST_F(GameStateTransitionTableTest, RefboxSignalsEndGame)
{
    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_STOPPED),
        RefboxSignalEnum::END_GAME,
        GameState(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));

    expect_gamestate(
        GameState(GoverningGameState::NEUTRAL_PLAYING),
        RefboxSignalEnum::END_GAME,
        GameState(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_PREPARING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::END_GAME,
        GameState(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));

    expect_gamestate(
        GameState(GoverningGameState::SETPIECE_EXECUTING, GoverningMatchState::IN_MATCH, SetpieceOwner::OWN, SetpieceType::PENALTY),
        RefboxSignalEnum::END_GAME,
        GameState(GoverningGameState::NEUTRAL_STOPPED, GoverningMatchState::OUT_OF_MATCH, SetpieceOwner::NONE, SetpieceType::NONE));
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
