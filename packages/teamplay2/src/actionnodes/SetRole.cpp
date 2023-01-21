// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/actionnodes/SetRole.hpp"

#include "int/stores/DiagnosticsStore.hpp"
#include "int/stores/RobotStore.hpp"

void SetRole::parseArguments()
{
    // Parse role
    auto res = getInput<teamplay::RoleEnum>("role");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [role]:", res.error());
    }
    _role = res.value();
}

behTreeReturnEnum SetRole::execute(bool onStarted)
{
    parseArguments();

    behTreeReturnEnum returnVal = behTreeReturnEnum::PASSED;

    teamplay::RobotStore::getInstance().setOwnRobotRole(_role);

    std::ostringstream oss;
    oss << "role: " << roleEnumMapping.right.at(_role);
    oss << "returning: " << behTreeReturnEnumMapping.right.at(returnVal);
    TRACE_FUNCTION(oss.str().c_str());

    teamplay::DiagnosticsStore::getDiagnostics().action = "SetRole";

    return returnVal;
}

BT::NodeStatus SetRole::onStart()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(true) );
}

BT::NodeStatus SetRole::onRunning()
{
    TRACE_FUNCTION("");

    return nodeStatusMapping.left.at( execute(false) );
}

void SetRole::onHalted()
{
}