// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPassToTarget.cpp
 *
 *  Created on: Nov 26, 2017
 *      Author: Jan Feitsma
 
The action passToTarget is derived from shootAtTarget, to make use of the same phases and timings (e.g. ballHandler release before shoot).
Only the configurables are different, as well as the shoot power (determined by shootPlanning).
 */

#include "../include/int/MP_ActionPassToTarget.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"

MP_ActionPassToTarget::MP_ActionPassToTarget()
{
    TRACE("init start");
    initialize();
    TRACE("init end");
}

void MP_ActionPassToTarget::getSpecificConfig()
{
    _accuracy = getConfig().passToTargetConfig.accuracy;
    _timeout = getConfig().passToTargetConfig.timeout;
    _actionName = "PASS";
}

void MP_ActionPassToTarget::unpackParameters()
{
    _shootTargetPos.x = boost::lexical_cast<float>(_params.at(0));
    _shootTargetPos.y = boost::lexical_cast<float>(_params.at(1));
    _shootTargetPos.z = 0.0;
    _shootType = actionTypeEnum::PASS;
}

void MP_ActionPassToTarget::executeShot()
{
    TRACE_FUNCTION("");
    TRACE("Executing pass");
    _rtdbOutput->setShootSetpoint(shootPhaseEnum::SHOOT, shootTypeEnum::PASS, Position2D(_distance, 0, 0));
}

