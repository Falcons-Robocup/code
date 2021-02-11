// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameStateTransitionTable.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATETRANSITIONTABLE_HPP_
#define GAMESTATETRANSITIONTABLE_HPP_

#include "int/types/cRefboxSignalTypes.hpp"
#include "int/types/gameState.hpp"


namespace teamplay
{

class gameStateTransitionTable {
public:
    static gameStateTransitionTable& getInstance()
    {
        static gameStateTransitionTable instance;
        return instance;
    }

    virtual const gameState calculateNewGameState(const gameState& oldGameState, const refboxSignalEnum& refboxSignal) const;

private:
    gameStateTransitionTable();
    virtual ~gameStateTransitionTable();
    gameStateTransitionTable(gameStateTransitionTable const&); // Don't implement
    void operator= (gameStateTransitionTable const&); // Don't implement
};


} /* namespace teamplay */

#endif /* GAMESTATETRANSITIONTABLE_HPP_ */
