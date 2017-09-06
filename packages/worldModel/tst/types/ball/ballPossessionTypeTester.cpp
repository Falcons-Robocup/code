 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballPossessionTypeTester.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>

#include "int/types/ball/ballPossessionType.hpp"

TEST(possessionTester, defaultNoPossession)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possession(1);

	/*
	 * Execution
	 */


	/*
	 * Verification
	 */
	EXPECT_TRUE(possession.hasBallPossession() == false);
}

TEST(possessionTester, visionOnlyNoPossession)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possession(1);

	/*
	 * Execution
	 */
	possession.claimBallByCamera();


	/*
	 * Verification
	 */
	EXPECT_TRUE(possession.hasBallPossession() == false);
}

TEST(possessionTester, ballHandlersOnlyPossession)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possession(1);

	/*
	 * Execution
	 */
	possession.claimBallByBallHandlers(1.0, 2.0);


	/*
	 * Verification
	 */
	EXPECT_TRUE(possession.hasBallPossession() == false);
}

TEST(possessionTester, verifyValidPossession)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possession(1);

	/*
	 * Execution
	 */
	possession.claimBallByBallHandlers(1.0, 2.0);
	possession.claimBallByCamera();


	/*
	 * Verification
	 */
	float x = 0.0;
	float y = 0.0;
	possession.getClaimedPosition(x,y);

	EXPECT_TRUE(possession.hasBallPossession() == true);
	EXPECT_FLOAT_EQ( 1.0f, x);
	EXPECT_FLOAT_EQ( 2.0f, y);
}

TEST(possessionTester, verifyReleasingPossession)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possession(1);

	/*
	 * Execution
	 */
	possession.claimBallByBallHandlers(1.0, 2.0);
	possession.claimBallByCamera();
	EXPECT_TRUE(possession.hasBallPossession() == true);

	possession.releaseBallByBallHandlers();

	/*
	 * Verification
	 */
	EXPECT_TRUE(possession.hasBallPossession() == false);
}

TEST(possessionTester, verifyRobotID)
{
	/*
	 * Setup
	 */
	ballPossessionClass_t possessionID1(1);
	ballPossessionClass_t possessionID4(4);

	/*
	 * Execution
	 */


	/*
	 * Verification
	 */
	uint8_t robotID1 = possessionID1.getRobotID();
	uint8_t robotID4 = possessionID4.getRobotID();

	EXPECT_EQ(robotID1, 1);
	EXPECT_EQ(robotID4, 4);
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
