// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleStimulatePassing.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

#include "int/rules/ruleStimulatePassing.hpp"
#include "int/stores/configurationStore.hpp"
#include "tracing.hpp"

namespace teamplay {

ruleStimulatePassing::ruleStimulatePassing()
{
    _robotsThatClaimedBall.clear();
}

ruleStimulatePassing::~ruleStimulatePassing()
{
}

void ruleStimulatePassing::resetRule()
{
    _robotsThatClaimedBall.clear();
}

void ruleStimulatePassing::robotClaimsBall(const int robot_id)
{
    _robotsThatClaimedBall.insert(robot_id);
}

bool ruleStimulatePassing::isRuleValid() const
{
    TRACE("Number of robots that claimed ball is ") << std::to_string(_robotsThatClaimedBall.size());
    return (_robotsThatClaimedBall.size() >= 2);
}

} /* namespace teamplay */
