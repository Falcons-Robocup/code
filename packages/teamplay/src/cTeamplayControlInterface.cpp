// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTeamplayControlInterface.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Jan Feitsma
 */

#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "int/cTeamplayControlInterface.hpp"
#include "int/types/cDecisionTreeTypes.hpp"
#include "tracing.hpp"

cTeamplayControlInterface::cTeamplayControlInterface()
{
    TRACE("constructing cTeamplayControlInterface");
}

void cTeamplayControlInterface::getOverrideState(tpOverrideState &overrideState)
{
    if (!_rtdbAdapter.getOverrideState(overrideState))
    {
        // either no data present, or too old -- clear it
        reset(overrideState);
    }
}

void cTeamplayControlInterface::setOverrideResult(tpOverrideResult const &overrideResult)
{
    _rtdbAdapter.setOverrideResult(overrideResult);
}

void cTeamplayControlInterface::reset(tpOverrideState &overrideState)
{
    overrideState.active = false;
    overrideState.level = tpOverrideLevelEnum::INVALID;
    overrideState.treeValue = treeEnum::INVALID;
    overrideState.tpAction = tpActionEnum::INVALID;
    overrideState.params.clear();
    overrideState.mpAction.action = actionTypeEnum::UNKNOWN;
}

