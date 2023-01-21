// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRtdbAdapterRefboxSignals.cpp
 *
 *  Created on: Oct 21, 2018
 *      Author: Jan Feitsma
 */

/* Own include */
#include "ext/cRtdbAdapterRefboxSignals.hpp"

/* Other teamplay includes */
#include "int/gameStateManager.hpp"

/* Falcons includes */
#include "falconsCommon.hpp" //getTeamChar()
#include "tracing.hpp"

using namespace std;

cRtdbAdapterRefboxSignals::cRtdbAdapterRefboxSignals()
{
    try
    {
        // setup RTDB connection, reading state from coach (id 0)
        _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(0, getTeamChar());
    }
    catch (exception &e)
    {
        //TODO TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void cRtdbAdapterRefboxSignals::update()
{
    try
    {
        int r = 0;
        int life = 0;
        if (_rtdb != NULL)
        {
            T_MATCH_STATE m;
            r = _rtdb->get(MATCH_STATE, &m, life, 0);
            if (r == RTDB2_SUCCESS)
            {
                std::string dataAsString = m.lastRefboxCommand;
                // e.g. "KICKOFF_OWN"
                std::string refboxSig = dataAsString;
                TRACE("command: %s", refboxSig.c_str());
                // special case: refbox protocol uses "DROP_BALL", but teamplay still uses "DROPPED_BALL"
                if (refboxSig == "DROP_BALL")
                {
                    refboxSig = "DROPPED_BALL";
                }
                // special case: substitution command takes arguments
                std::string refboxSignalArgument;
                std::string searchStr = "SUBSTITUTION_OWN";
                size_t pos = refboxSig.find(searchStr);
                if (pos == 0)
                {
                    // split argument out of main signal, so enum can be calculated
                    refboxSignalArgument = refboxSig.substr(1+std::string(searchStr).size());
                    refboxSig = searchStr;
                }
                // Convert string to refbox signal enum
                refboxSignalEnum refboxSignal = refboxSignalMapping[refboxSig];
                // Store
                teamplay::gameStateManager::getInstance().refBoxSignalReceived(refboxSignal, refboxSignalArgument);
            }
        }
    }
    catch (exception &e)
    {
        //TODO TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

