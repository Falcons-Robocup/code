// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/PositionBehindPOI.hpp"

#include "int/stores/DiagnosticsStore.hpp"

void PositionBehindPOI::parseArguments()
{
    // Parse from
    auto res = getInput<Point2D>("from");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [from]:", res.error());
    }
    _from = res.value();

    // Parse to
    auto res2 = getInput<Point2D>("to");
    if (!res2)
    {
        throw BT::RuntimeError("error reading port [to]:", res2.error());
    }
    _to = res2.value();

    // Parse distance
    auto res3 = getInput<float>("distance");
    if (!res3)
    {
        throw BT::RuntimeError("error reading port [distance]:", res3.error());
    }
    _distance = res3.value();

    // Parse motionType
    auto res4 = getInput<motionTypeEnum>("motionType");
    if (!res4)
    {
        throw BT::RuntimeError("error reading port [motionType]:", res4.error());
    }
    _motionType = res4.value();

    // Parse distanceThreshold
    auto res5 = getInput<float>("distanceThreshold");
    if (!res5)
    {
        throw BT::RuntimeError("error reading port [distanceThreshold]:", res5.error());
    }
    _distanceThreshold = res5.value();
}

behTreeReturnEnum PositionBehindPOI::execute(bool onStarted)
{
    parseArguments();

    Vector2D targetVec = ( ( (_to - _from).size() + _distance ) / (_to - _from).size() ) * (_to - _from) + _from;
    _targetPos = Point2D(targetVec.x, targetVec.y);

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

    std::ostringstream oss;
    oss << "from: " << _from << "\n";
    oss << "to: " << _to << "\n";
    oss << "distance: " << _distance << "\n";
    oss << "distanceThreshold: " << _distanceThreshold << "\n";
    oss << "motionType: " << motionTypeEnumMapping.right.at(_motionType) << "\n";
    oss << "targetPos: " << _targetPos << "\n";

    if (positionReached(_targetPos.x, _targetPos.y, _distanceThreshold))
    {
        returnVal = behTreeReturnEnum::PASSED;
        oss << "targetReached: true\n";
    }
    else
    {
        oss << "targetReached: false\n";
    }
    
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    moveTo(_targetPos, _motionType);

    teamplay::DiagnosticsStore::getDiagnostics().action = "PositionBeforePOI";

    return returnVal;
}

BT::NodeStatus PositionBehindPOI::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus PositionBehindPOI::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void PositionBehindPOI::onHalted()
{
}