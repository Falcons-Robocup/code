// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameStateStore.hpp
 *
 *  Created on: Jul 26, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATESTORE_HPP_
#define GAMESTATESTORE_HPP_

#include "int/types/gameState.hpp"


namespace teamplay
{

class gameStateStore {
public:
    static gameStateStore& getInstance()
    {
        static gameStateStore instance;
        return instance;
    }

    virtual const gameState& getGameState() const;
    virtual treeEnum getGameState_treeEnum() const;
    virtual void updateGameState(const gameState &);
    virtual void updateGameState(const governingGameState &);
    virtual void updateGameState(const governingGameState &, const setpieceOwner &, const setpieceType &);
    virtual void updateGameState(const treeEnum &);

private:
    gameStateStore();
    virtual ~gameStateStore();
    gameStateStore(gameStateStore const&); // Don't implement
    void operator= (gameStateStore const&); // Don't implement

    gameState _theGameState;
};


} /* namespace teamplay */

#endif /* GAMESTATESTORE_HPP_ */
