// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleStimulatePassing.hpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

#ifndef RULESTIMULATEPASSING_HPP_
#define RULESTIMULATEPASSING_HPP_

#include <set>

namespace teamplay {

class RuleStimulatePassing
{
public:
    static RuleStimulatePassing& getInstance()
    {
        static RuleStimulatePassing instance;
        return instance;
    }

    virtual void resetRule();
    virtual void robotClaimsBall(int robot_id);
    virtual bool isRuleValid() const;

private:
    RuleStimulatePassing();
    virtual ~RuleStimulatePassing();

    std::set<int> _robotsThatClaimedBall;
};

} /* namespace teamplay */

#endif /* RULESTIMULATEPASSING_HPP_ */
