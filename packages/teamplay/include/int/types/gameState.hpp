 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    DROPPED_BALL
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
    virtual bool isCornerSetPiece() const;
    virtual bool isDroppedBallSetPiece() const;
    virtual bool isFreekickSetPiece() const;
    virtual bool isGoalkickSetPiece() const;
    virtual bool isKickoffSetPiece() const;
    virtual bool isPenaltySetPiece() const;
    virtual bool isSidelineSetPiece() const;
    virtual bool isThrowinSetPiece() const;
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
