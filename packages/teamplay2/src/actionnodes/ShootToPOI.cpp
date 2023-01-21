// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/ShootToPOI.hpp"

#include "int/stores/DiagnosticsStore.hpp"

void ShootToPOI::parseArguments()
{
    // Parse poi
    auto res = getInput<Point2D>("poi");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [poi]:", res.error());
    }
    _poi = res.value();

    // Parse shootType
    auto res2 = getInput<tpShootTypeEnum>("shootType");
    if (!res2)
    {
        throw BT::RuntimeError("error reading port [shootType]:", res2.error());
    }
    _shootType = res2.value();
}

behTreeReturnEnum ShootToPOI::execute(bool onStarted)
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
    
    switch(_shootType)
    {
        case tpShootTypeEnum::SHOOT:
        {
            shoot(_poi);
            break;
        }
        case tpShootTypeEnum::LOB_SHOT:
        {
            lobShot(_poi);
            break;
        }
        case tpShootTypeEnum::PASS:
        {
            pass(_poi);
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown tpShootTypeEnum");
        }
    }

    std::ostringstream oss;
    oss << "poi: " << _poi << "\n";
    oss << "shootType: " << shootTypeMapping.right.at(_shootType) << "\n";
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    teamplay::DiagnosticsStore::getDiagnostics().action = "ShootToPOI";

    return returnVal;
}

BT::NodeStatus ShootToPOI::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus ShootToPOI::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void ShootToPOI::onHalted()
{
}