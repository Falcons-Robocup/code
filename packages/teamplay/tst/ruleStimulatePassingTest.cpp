// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleStimulatePassingTest.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/rules/ruleStimulatePassing.hpp"


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
    EXPECT_FALSE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterOneBallPossessor)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    EXPECT_FALSE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterOneBallPossessorClaimingRepeatedly)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    EXPECT_FALSE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterTwoBallPossessors)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    EXPECT_TRUE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterTwoBallPossessorsClaimingRepeatedly)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    EXPECT_TRUE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsValidAfterMultipleBallPossessors)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(2);
    ruleStimulatePassing::getInstance().robotClaimsBall(6);
    EXPECT_TRUE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterReset)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(2);
    ruleStimulatePassing::getInstance().robotClaimsBall(6);
    ruleStimulatePassing::getInstance().resetRule();
    EXPECT_FALSE(ruleStimulatePassing::getInstance().isRuleValid());
}

TEST_F(RuleStimulatePassingTest, RuleIsNotValidAfterResetFollowedByAClaim)
{
    ruleStimulatePassing::getInstance().robotClaimsBall(3);
    ruleStimulatePassing::getInstance().robotClaimsBall(4);
    ruleStimulatePassing::getInstance().robotClaimsBall(2);
    ruleStimulatePassing::getInstance().robotClaimsBall(6);
    ruleStimulatePassing::getInstance().resetRule();
    ruleStimulatePassing::getInstance().robotClaimsBall(2);
    EXPECT_FALSE(ruleStimulatePassing::getInstance().isRuleValid());
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
