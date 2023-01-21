// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleStimulatePassingTest.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/rules/RuleStimulatePassing.hpp"


using namespace teamplay;

class RuleStimulatePassingTest : public TeamplayTest
{
public:
    RuleStimulatePassingTest()
    {
    }
};

TEST_F(RuleStimulatePassingTest, RuleIsInitiallyNotValid)
{
    EXPECT_FALSE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterOneBallPossessor)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    EXPECT_FALSE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterOneBallPossessorClaimingRepeatedly)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    EXPECT_FALSE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterTwoBallPossessors)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    EXPECT_TRUE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterTwoBallPossessorsClaimingRepeatedly)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    EXPECT_TRUE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterMultipleBallPossessors)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(2);
    RuleStimulatePassing::getInstance().robotClaimsBall(6);
    EXPECT_TRUE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterReset)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(2);
    RuleStimulatePassing::getInstance().robotClaimsBall(6);
    RuleStimulatePassing::getInstance().resetRule();
    EXPECT_FALSE(RuleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterResetFollowedByAClaim)
{
    RuleStimulatePassing::getInstance().robotClaimsBall(3);
    RuleStimulatePassing::getInstance().robotClaimsBall(4);
    RuleStimulatePassing::getInstance().robotClaimsBall(2);
    RuleStimulatePassing::getInstance().robotClaimsBall(6);
    RuleStimulatePassing::getInstance().resetRule();
    RuleStimulatePassing::getInstance().robotClaimsBall(2);
    EXPECT_FALSE(RuleStimulatePassing::getInstance().isRuleValid());
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
