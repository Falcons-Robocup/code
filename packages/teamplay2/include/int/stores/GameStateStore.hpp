// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateStore.hpp
 *
 *  Created on: Jul 26, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATESTORE_HPP_
#define GAMESTATESTORE_HPP_

#include "int/types/GameState.hpp"


namespace teamplay
{

class GameStateStore {
public:
    static GameStateStore& getInstance()
    {
        static GameStateStore instance;
        return instance;
    }

    virtual const GameState& getGameState() const;
    virtual void updateGameState(const GameState &);
    virtual void updateGameState(const GoverningGameState &);
    virtual void updateGameState(const GoverningGameState &, const GoverningMatchState &, const SetpieceOwner &, const SetpieceType &);

private:
    GameStateStore();
    virtual ~GameStateStore();
    GameStateStore(GameStateStore const&); // Don't implement
    void operator= (GameStateStore const&); // Don't implement

    GameState _theGameState;
};


} /* namespace teamplay */

#endif /* GAMESTATESTORE_HPP_ */
