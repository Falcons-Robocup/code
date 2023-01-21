// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBAdapterRefboxSignals.hpp
 *
 *  Created on: Oct 21, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBADAPTERREFBOXSIGNALS_HPP_
#define CRTDBADAPTERREFBOXSIGNALS_HPP_

#include "MTPAdapter.hpp"


/*!
 * \brief The adapter class to get the Refbox Signals inputs for Teamplay
 *
 * The class RTDBAdapterRefboxSignals is a singleton class which reads shared RTDB state
 * coming from coach, and communicates the signal to the game state manager.
 */
class RTDBAdapterRefboxSignals
{
public:
    /*!
    * \brief Obtain an instance of this singleton class
    */
    static RTDBAdapterRefboxSignals& getInstance()
    {
        static RTDBAdapterRefboxSignals instance; 
        return instance;
    }
    
    void setMTP(MixedTeamProtocolAdapter* mtp);
    void update();

private:
    RTDBAdapterRefboxSignals();
    MixedTeamProtocolAdapter* _mtpAdapter = NULL;
        
};

#endif

