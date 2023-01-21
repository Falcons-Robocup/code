// Copyright 2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/decoratornodes/AvoidPOI.hpp"

#include "tracing.hpp"

#include "int/stores/BallStore.hpp"

void AvoidPOI::parseArguments()
{
    // Parse poi
    auto res = getInput<Point2D>("poi");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [poi]:", res.error());
    }
    _poi = res.value();
}


BT::NodeStatus AvoidPOI::tick()
{
    TRACE_FUNCTION("");

    setStatus(BT::NodeStatus::RUNNING);

    teamplay::BallStore::getBall().avoid();

    return child_node_->executeTick();
}