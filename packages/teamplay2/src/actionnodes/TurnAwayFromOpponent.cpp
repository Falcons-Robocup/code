// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/TurnAwayFromOpponent.hpp"

#include "int/stores/DiagnosticsStore.hpp"

void TurnAwayFromOpponent::parseArguments()
{
    // Parse target
    auto res = getInput<Point2D>("target");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    _target = res.value();
}

behTreeReturnEnum TurnAwayFromOpponent::execute(bool onStarted)
{
    parseArguments();

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
    
    turnAwayFromOpponent(_target);

    std::ostringstream oss;
    oss << "target: " << _target;
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    teamplay::DiagnosticsStore::getDiagnostics().action = "TurnAwayFromOpponent";

    return returnVal;
}

BT::NodeStatus TurnAwayFromOpponent::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus TurnAwayFromOpponent::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void TurnAwayFromOpponent::onHalted()
{
}