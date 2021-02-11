// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameState.hpp
 *
 *  Created on: Jul 25, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATE_HPP_
#define GAMESTATE_HPP_

#include <string>

#include "int/types/cGameStateTypes.hpp"

namespace teamplay
{

enum class governingGameState {
    INVALID,
    NEUTRAL_STOPPED,
    NEUTRAL_PLAYING,
    SETPIECE_PREPARING,
    SETPIECE_EXECUTING
};

enum class governingMatchState {
    IN_MATCH,
    OUT_OF_MATCH
};

enum class setpieceType {
    NONE,
    KICKOFF,
    FREEKICK,
    GOALKICK,
    THROWIN,
    CORNER,
    PENALTY,
    DROPPED_BALL,
    PARKING
};

enum class setpieceOwner {
    NONE,
    OWN,
    OPPONENT
};

class gameState {
public:
    gameState ();
    gameState (const governingGameState &);
    gameState (const governingGameState &, const governingMatchState &, const setpieceOwner &, const setpieceType &);
    gameState (const treeEnum &);
    virtual ~gameState();

    virtual bool operator== (const gameState&) const;
    virtual bool operator!= (const gameState&) const;

    virtual std::string toString() const;
    virtual treeEnum toTreeEnum() const;

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
    virtual bool isInMatch() const;
    virtual bool isStopped() const;

private:
    governingGameState _governingGameState;
    governingMatchState _governingMatchState;
    setpieceType _setpieceType;
    setpieceOwner _setpieceOwner;

    /* The function below is private for a good reason: the gameState class is immutable.
     * Clients can destroy and create a new gameState instance if they wish to change the game state.
     */
    virtual void setGameState(const governingGameState &, const governingMatchState &, const setpieceOwner &, const setpieceType &);
    virtual std::string toString(const governingGameState &, const governingMatchState &, const setpieceOwner &, const setpieceType &) const;
};

} /* namespace teamplay */

#endif /* GAMESTATE_HPP_ */
