// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRtdbAdapterRefboxSignals.hpp
 *
 *  Created on: Oct 21, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBADAPTERREFBOXSIGNALS_HPP_
#define CRTDBADAPTERREFBOXSIGNALS_HPP_

#include "FalconsRtDB2.hpp"


/*!
 * \brief The adapter class to get the Refbox Signals inputs for Teamplay
 *
 * The class cRtdbAdapterRefboxSignals is a singleton class which reads shared RTDB state
 * coming from coach, and communicates the signal to the game state manager.
 */
class cRtdbAdapterRefboxSignals
{
public:
    /*!
    * \brief Obtain an instance of this singleton class
    */
    static cRtdbAdapterRefboxSignals& getInstance()
    {
        static cRtdbAdapterRefboxSignals instance; 
        return instance;
    }
    
    void update();

private:
    cRtdbAdapterRefboxSignals();
    RtDB2 *_rtdb = NULL;
        
};

#endif

