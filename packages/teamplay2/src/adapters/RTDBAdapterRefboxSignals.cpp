// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBAdapterRefboxSignals.cpp
 *
 *  Created on: Oct 21, 2018
 *      Author: Jan Feitsma
 */

/* Own include */
#include "int/adapters/RTDBAdapterRefboxSignals.hpp"

/* Other teamplay includes */
#include "int/gamestate/GameStateManager.hpp"

/* Falcons includes */
#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace std;

RTDBAdapterRefboxSignals::RTDBAdapterRefboxSignals()
{
}

void RTDBAdapterRefboxSignals::setMTP(MixedTeamProtocolAdapter* mtp)
{
    _mtpAdapter = mtp;
}

void RTDBAdapterRefboxSignals::update()
{
    TRACE_FUNCTION("");
    try
    {
        bool changed = false;
        std::string command;
        mtp::RefereeCommand::Arguments commandArgs;
        (*_mtpAdapter)->getRefboxCommand(changed, command, commandArgs);
        if (changed)
        {
            // convert string to enum
            if (refboxSignalMapping.count(command))
            {
                tprintf("new refbox command: %s", command.c_str());
                RefboxSignalEnum refboxSignalEnum = refboxSignalMapping.at(command);
                teamplay::GameStateManager::getInstance().refBoxSignalReceived(refboxSignalEnum, commandArgs);
            }
            else
            {
                tprintf("ignoring UNKNOWN new refbox command: %s", command.c_str());
            }
        }
    }
    catch(const std::exception& e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}