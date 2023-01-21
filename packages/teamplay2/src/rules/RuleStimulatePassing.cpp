// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleStimulatePassing.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

#include "int/rules/RuleStimulatePassing.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "tracing.hpp"

namespace teamplay {

RuleStimulatePassing::RuleStimulatePassing()
{
    _robotsThatClaimedBall.clear();
}

RuleStimulatePassing::~RuleStimulatePassing()
{
}

void RuleStimulatePassing::resetRule()
{
    _robotsThatClaimedBall.clear();
}

void RuleStimulatePassing::robotClaimsBall(const int robot_id)
{
    _robotsThatClaimedBall.insert(robot_id);
}

bool RuleStimulatePassing::isRuleValid() const
{
    TRACE("Number of robots that claimed Ball is ") << std::to_string(_robotsThatClaimedBall.size());
    return (_robotsThatClaimedBall.size() >= 2);
}

} /* namespace teamplay */
