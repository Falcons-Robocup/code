// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/PositionWithHeightmap.hpp"

#include "int/stores/DiagnosticsStore.hpp"
#include "int/stores/HeightMapStore.hpp"

void PositionWithHeightmap::parseArguments()
{
    // Parse heightmap
    auto res = getInput<CompositeHeightmapName>("heightmap");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [heightmap]:", res.error());
    }
    _heightmap = res.value();
}

behTreeReturnEnum PositionWithHeightmap::execute(bool onStarted)
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

    // Get optimal location from heightmap
    auto optimum = teamplay::HeightMapStore::getInstance().getOptimum(_heightmap);

    Point2D targetPos;
    targetPos.x = optimum.x;
    targetPos.y = optimum.y;

    std::ostringstream oss;
    oss << "targetPos: " << targetPos << "\n";
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    moveTo(targetPos);

    teamplay::DiagnosticsStore::getDiagnostics().action = "PositionWithHeightMap";
    teamplay::DiagnosticsStore::getDiagnostics().activeHeightmap = _heightmap;

    return returnVal;
}

BT::NodeStatus PositionWithHeightmap::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus PositionWithHeightmap::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void PositionWithHeightmap::onHalted()
{
}