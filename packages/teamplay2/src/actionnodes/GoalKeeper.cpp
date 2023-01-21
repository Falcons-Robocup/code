// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/GoalKeeper.hpp"

#include "int/stores/DiagnosticsStore.hpp"

behTreeReturnEnum GoalKeeper::execute(bool onStarted)
{
    behTreeReturnEnum returnVal;

    if (onStarted)
    {
        returnVal = behTreeReturnEnum::RUNNING;
    }
    else
    {
        // Get result from previous tick
        returnVal = getActionResult();
    }
    
    keeperMove();

    teamplay::DiagnosticsStore::getDiagnostics().action = "GoalKeeper";

    return returnVal;
}

BT::NodeStatus GoalKeeper::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus GoalKeeper::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void GoalKeeper::onHalted()
{
}