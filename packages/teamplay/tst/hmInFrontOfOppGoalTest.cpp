 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmInFrontOfOppGoalTest.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmInFrontOfOppGoal.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"


/* Testing the 'in front of opponent goal' heightmap */

class hmInFrontOfOppGoalTest : public TeamplayTest
{
public:
    hmInFrontOfOppGoalTest()
    {
        ballStore::getInstance().getBall().setPosition(Point3D(1.0, 1.0, 1.0));
    }
    hmInFrontOfOppGoal _hmInFrontOfOppGoal;
    parameterMap_t _parameters;
};

TEST_F(hmInFrontOfOppGoalTest, onBothSides)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "both";
    _hmInFrontOfOppGoal.refine(_parameters);

    _hmInFrontOfOppGoal.generateJPG("tst_hmInFrontOfOppGoal");
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithBall)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "withBall";
    _hmInFrontOfOppGoal.refine(_parameters);

    _hmInFrontOfOppGoal.generateJPG("tst_hmInFrontOfOppGoalOnSideWithBall");
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithoutBall)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "withoutBall";
    _hmInFrontOfOppGoal.refine(_parameters);

    _hmInFrontOfOppGoal.generateJPG("tst_hmInFrontOfOppGoalOnSideWithoutBall");
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
