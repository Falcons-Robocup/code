// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateTransitionTable.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATETRANSITIONTABLE_HPP_
#define GAMESTATETRANSITIONTABLE_HPP_

#include "int/types/RefboxSignalTypes.hpp"
#include "int/types/GameState.hpp"


namespace teamplay
{

class GameStateTransitionTable {
public:
    static GameStateTransitionTable& getInstance()
    {
        static GameStateTransitionTable instance;
        return instance;
    }

    virtual const GameState calculateNewGameState(const GameState& oldGameState, const RefboxSignalEnum& refboxSignal) const;

private:
    GameStateTransitionTable();
    virtual ~GameStateTransitionTable();
    GameStateTransitionTable(GameStateTransitionTable const&); // Don't implement
    void operator= (GameStateTransitionTable const&); // Don't implement
};


} /* namespace teamplay */

#endif /* GAMESTATETRANSITIONTABLE_HPP_ */
