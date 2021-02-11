// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleStimulatePassing.hpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

#ifndef RULESTIMULATEPASSING_HPP_
#define RULESTIMULATEPASSING_HPP_

#include <set>

namespace teamplay {

class ruleStimulatePassing
{
public:
    static ruleStimulatePassing& getInstance()
    {
        static ruleStimulatePassing instance;
        return instance;
    }

    virtual void resetRule();
    virtual void robotClaimsBall(int robot_id);
    virtual bool isRuleValid() const;

private:
    ruleStimulatePassing();
    virtual ~ruleStimulatePassing();

    std::set<int> _robotsThatClaimedBall;
};

} /* namespace teamplay */

#endif /* RULESTIMULATEPASSING_HPP_ */
