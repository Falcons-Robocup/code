// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBDiagnosticsAdapter.cpp
 *
 *  Created on: Jun 26, 2022
 *      Author: Edwin Schreuder
 */

#include "int/adapters/RTDBDiagnosticsAdapter.hpp"

#include "int/stores/DiagnosticsStore.hpp"
#include "int/stores/GameStateStore.hpp"
#include "int/stores/HeightMapStore.hpp"
#include "int/stores/RobotStore.hpp"

#include "diagTeamplay.hpp"

#include "cDiagnostics.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

tpRTDBDiagnosticsAdapter::tpRTDBDiagnosticsAdapter()
{
    TRACE(">");
    int _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
    TRACE("<");
}

void tpRTDBDiagnosticsAdapter::setDiagnosticsData()
{
    TRACE_FUNCTION("");

    try
    {
        diagTeamplay diagMsg;

        // TODO: Fill up teamplay diagnostics data
        // diagMsg.trees.push_back(node._name);

        // // Copy the names of nodes 1, 2, 3 and N for easier diagnostics
        // if (diagMsg.trees.size() >= 4)
        // {
        //     diagMsg.state = diagMsg.trees.front();
        //     diagMsg.role = diagMsg.trees.at(1);
        //     diagMsg.behavior = diagMsg.trees.at(2);
        //     diagMsg.action = diagMsg.trees.back();
        // }

        diagMsg.action = teamplay::DiagnosticsStore::getDiagnostics().action;
        teamplay::Role role(teamplay::RobotStore::getInstance().getOwnRobot().getRole());
        diagMsg.role = role.str();
        diagMsg.state = teamplay::GameStateStore::getInstance().getGameState().toString();
        diagMsg.activeHeightmap = teamplay::DiagnosticsStore::getDiagnostics().activeHeightmap;
        // diagMsg.behavior = teamplay::DiagnosticsStore::getDiagnostics().getBehavior();
        // diagMsg.trees;
        // teamplay::HeightMapStore.getInstance().getDescriptions();

        // Copy the contents of the diagnostics store into the diagnostics message
        diagMsg.aiming = teamplay::DiagnosticsStore::getDiagnostics().aiming;
        auto shootTarget = teamplay::DiagnosticsStore::getDiagnostics().shootTarget;
        diagMsg.shootTargetX = shootTarget.x;
        diagMsg.shootTargetY = shootTarget.y;

        // Send diagnostics message and clear the diagnostics store for the next iteration
        if (_rtdb != NULL)
        {
            _rtdb->put(DIAG_TEAMPLAY, &diagMsg); // TODO: move to diagnostics store? But, it would not be a store anymore .. diagnosticsAdapter then?
        }

        // Clear diagnostics so it is filled with new data upon next iteration
        teamplay::DiagnosticsStore::getInstance().clear();
    }
    catch (std::exception& e)
    {
        TRACE_ERROR("Caught exception while storing diagnostics data: %s", e.what());
        throw std::runtime_error(std::string("Caught exception while storing diagnostics data: ") + e.what());
    }
}
