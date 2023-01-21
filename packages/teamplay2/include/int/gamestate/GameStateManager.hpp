// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateManager.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATEMANAGER_HPP_
#define GAMESTATEMANAGER_HPP_

#include "int/types/RefboxSignalTypes.hpp"


namespace teamplay
{

class GameStateManager {
public:
    static GameStateManager& getInstance()
    {
        static GameStateManager instance;
        return instance;
    }

    /*! \brief Check whether ball movement or elapsed time is sufficient. If yes, update the gamestate. */
    virtual void refreshGameState();

    /*! \brief Update the gamestate according to the refbox signal. */
    virtual void refBoxSignalReceived(const RefboxSignalEnum &, const RefboxSignalArguments& refboxSignalArguments);

private:
    GameStateManager();
    virtual ~GameStateManager();
    GameStateManager(GameStateManager const&); // Don't implement
    void operator= (GameStateManager const&); // Don't implement
};


} /* namespace teamplay */

#endif /* GAMESTATEMANAGER_HPP_ */
