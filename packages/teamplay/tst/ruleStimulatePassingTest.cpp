 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
