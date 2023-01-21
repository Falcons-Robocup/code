// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/GetBall.hpp"

#include "int/stores/DiagnosticsStore.hpp"

void GetBall::parseArguments()
{
    // Parse motionType
    auto res = getInput<motionTypeEnum>("motionType");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [motionType]:", res.error());
    }
    _motionType = res.value();
}

behTreeReturnEnum GetBall::execute(bool onStarted)
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
    
    getBall(_motionType);

    std::ostringstream oss;
    oss << "motionType: " << motionTypeEnumMapping.right.at(_motionType) << "\n";
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    teamplay::DiagnosticsStore::getDiagnostics().action = "GetBall";

    return returnVal;
}

BT::NodeStatus GetBall::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus GetBall::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void GetBall::onHalted()
{
}