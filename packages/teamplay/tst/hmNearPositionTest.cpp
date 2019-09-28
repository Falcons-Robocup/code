 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmNearPosition.cpp
 *
 *  Created on: June 5, 2018
 *      Author: Jan Feitsma
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmNearPosition.hpp"

/* SUT dependencies */
#include "int/stores/robotStore.hpp"


/* Testing the 'near position' heightmap */

class hmNearPositionTest : public TeamplayTest
{
public:
    hmNearPositionTest()
    {
    }
    hmNearPosition _hmNearPosition;
    parameterMap_t _parameters;
};

TEST_F(hmNearPositionTest, generateJPG)
{
    _hmNearPosition.precalculate();
    
    _parameters["positionX"] = "1.0";
    _parameters["positionY"] = "6.0";
    _hmNearPosition.refine(_parameters);
    
    auto optimum = _hmNearPosition.getOptimum();

    EXPECT_EQ(1.0, optimum.x);
    EXPECT_EQ(6.0, optimum.y);
    
    _hmNearPosition.generateJPG("tst_hmNearPosition");
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
