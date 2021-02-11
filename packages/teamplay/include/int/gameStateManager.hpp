// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameStateManager.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef GAMESTATEMANAGER_HPP_
#define GAMESTATEMANAGER_HPP_

#include "int/types/cRefboxSignalTypes.hpp"


namespace teamplay
{

class gameStateManager {
public:
    static gameStateManager& getInstance()
    {
        static gameStateManager instance;
        return instance;
    }

    /*! \brief Check whether ball movement or elapsed time is sufficient. If yes, update the gamestate. */
    virtual void refreshGameState();

    /*! \brief Update the gamestate according to the refbox signal. */
    virtual void refBoxSignalReceived(const refboxSignalEnum &, std::string refboxSignalArgument = "");

private:
    gameStateManager();
    virtual ~gameStateManager();
    gameStateManager(gameStateManager const&); // Don't implement
    void operator= (gameStateManager const&); // Don't implement
};


} /* namespace teamplay */

#endif /* GAMESTATEMANAGER_HPP_ */
