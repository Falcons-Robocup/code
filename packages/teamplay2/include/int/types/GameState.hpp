// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameState.hpp
 *
 *  Created on: Jul 25, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATE_HPP_
#define GAMESTATE_HPP_

#include <string>

namespace teamplay
{

enum class GoverningGameState {
    INVALID,
    NEUTRAL_STOPPED,
    NEUTRAL_PLAYING,
    SETPIECE_PREPARING,
    SETPIECE_EXECUTING
};

enum class GoverningMatchState {
    IN_MATCH,
    OUT_OF_MATCH
};

enum class SetpieceType {
    NONE,
    KICKOFF,
    FREEKICK,
    GOALKICK,
    THROWIN,
    CORNER,
    PENALTY,
    DROPPED_BALL,
    PARKING,
    SHOWING_ALIVE,
};

enum class SetpieceOwner {
    NONE,
    OWN,
    OPPONENT
};


class GameState {
public:
    GameState ();
    GameState (const GoverningGameState &);
    GameState (const GoverningGameState &, const GoverningMatchState &, const SetpieceOwner &, const SetpieceType &);
    virtual ~GameState();

    virtual bool operator== (const GameState&) const;
    virtual bool operator!= (const GameState&) const;
    virtual bool operator< (const GameState&) const;

    virtual std::string toString() const;
    virtual void fromString(const std::string&);

    virtual bool isSetPiece() const;
    virtual bool isOwnSetPiece() const;
    virtual bool isPrepareSetPiece() const;
    virtual bool isExecuteSetPiece() const;
    virtual bool isCornerSetPiece() const;
    virtual bool isDroppedBallSetPiece() const;
    virtual bool isFreekickSetPiece() const;
    virtual bool isGoalkickSetPiece() const;
    virtual bool isKickoffSetPiece() const;
    virtual bool isPenaltySetPiece() const;
    virtual bool isThrowinSetPiece() const;
    virtual bool isParkingSetPiece() const;
    virtual bool isShowingAliveSetPiece() const;
    virtual bool isInMatch() const;
    virtual bool isStopped() const;

    virtual bool isValidGameState() const;

private:
    GoverningGameState _governingGameState;
    GoverningMatchState _governingMatchState;
    SetpieceType _setpieceType;
    SetpieceOwner _setpieceOwner;

    /* The function below is private for a good reason: the GameState class is immutable.
     * Clients can destroy and create a new GameState instance if they wish to change the game state.
     */
    virtual void setGameState(const GoverningGameState &, const GoverningMatchState &, const SetpieceOwner &, const SetpieceType &);
    virtual std::string toString(const GoverningGameState &, const GoverningMatchState &, const SetpieceOwner &, const SetpieceType &) const;

    virtual GoverningGameState fromStringGoverningGameState(const std::string&) const;
    virtual GoverningMatchState fromStringGoverningMatchState(const std::string&) const;
    virtual SetpieceType fromStringSetpieceType(const std::string&) const;
    virtual SetpieceOwner fromStringSetpieceOwner(const std::string&) const;
};

} /* namespace teamplay */

#endif /* GAMESTATE_HPP_ */
