// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/Move.hpp"

#include "int/stores/DiagnosticsStore.hpp"

void Move::parseArguments()
{
    // Parse target
    auto res = getInput<Point2D>("target");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    _target = res.value();

    // Parse motionType
    auto res2 = getInput<motionTypeEnum>("motionType");
    if (!res2)
    {
        throw BT::RuntimeError("error reading port [motionType]:", res2.error());
    }
    _motionType = res2.value();
}

behTreeReturnEnum Move::execute(bool onStarted)
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
    
    moveTo(_target, _motionType);

    std::ostringstream oss;
    oss << "targetPos: " << _target << "\n";
    oss << "motionType: " << motionTypeEnumMapping.right.at(_motionType) << "\n";
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    teamplay::DiagnosticsStore::getDiagnostics().action = "Move";

    return returnVal;
}

BT::NodeStatus Move::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus Move::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void Move::onHalted()
{
}