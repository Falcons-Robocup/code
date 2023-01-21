// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/InterceptBall.hpp"

#include "int/stores/DiagnosticsStore.hpp"

behTreeReturnEnum InterceptBall::execute(bool onStarted)
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
    
    interceptBall();

    teamplay::DiagnosticsStore::getDiagnostics().action = "InterceptBall";

    return returnVal;
}

BT::NodeStatus InterceptBall::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus InterceptBall::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void InterceptBall::onHalted()
{
}