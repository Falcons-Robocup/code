 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cObjectPathTest.cpp
 *
 *  Created on: Feb 28, 2016
 *      Author: Michel Koenen
 */

#include <gtest/gtest.h>
#include "int/utilities/trace.hpp"

#include "int/types/cPositionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"
#include "int/types/cBallLocationTypes.hpp"

#include "int/cObjectPath.hpp"


TEST(objectPath, ballLineIntersect1)
{
	/*
	 * Setup
	 */
	ballLocation ball;
	ball.position = Point3D( 0.0, 0.0, 0.0);
	ball.velocity = Point3D( 1.0, 1.0, 0.0);
	cObjectPath ballLine(  ball.position, ball.velocity );



	bool intersect;
	Point2D intersectionCoord;

	/*
	 * Execution
	 */
	intersect=ballLine.intersectCheck( Point2D(5.0, 3.0), Point2D(3.0, 5.0), intersectionCoord);

	/*
	 * Verification
	 */
	EXPECT_TRUE(intersect == true);
	EXPECT_FLOAT_EQ( 4.0f, intersectionCoord.x);
	EXPECT_FLOAT_EQ( 4.0f, intersectionCoord.y);
}

TEST(objectPath, ballLineIntersect2)
{
	/*
	 * Setup
	 */
	ballLocation ball;
	ball.position = Point3D( 0.0, 0.0, 0.0);
	ball.velocity = Point3D( -1.0, -1.0, 0.0);
	cObjectPath ballLine(  ball.position, ball.velocity );

	bool intersect;
	Point2D intersectionCoord;

	/*
	 * Execution
	 */
	intersect=ballLine.intersectCheck( Point2D(5.0, 3.0), Point2D(3.0, 5.0), intersectionCoord);

	/*
	 * Verification
	 */
	EXPECT_TRUE(intersect == false);
}

TEST(objectPath, ballLineIntercept1)
{
	/*
	 * Setup
	 */
	ballLocation ball;
	ball.position = Point3D(-3.0, 0.0, 0.0);
	ball.velocity = Point3D(-1.0,-1.0, 0.0);
	cObjectPath ballLine(  ball.position, ball.velocity );

	bool valid;
	Point2D interceptionCoord;

	/*
	 * Execution
	 */
	interceptionCoord=ballLine.getPerpendicularInterceptionPoint( Point2D(2.5, -2.0), valid);

	/*
	 * Verification
	 */
	EXPECT_FLOAT_EQ( -1.25f, interceptionCoord.x);
	EXPECT_FLOAT_EQ( 1.75f, interceptionCoord.y);
	EXPECT_FALSE( valid );
}

TEST(objectPath, ballLineIntercept2)
{
	/*
	 * Setup
	 */
	ballLocation ball;
	ball.position = Point3D(0.0, 0.0, 0.0);
	ball.velocity = Point3D(0.0, -0.5, 0.0);
	cObjectPath ballLine(  ball.position, ball.velocity );

	bool valid;
	Point2D interceptionCoord;

	/*
	 * Execution
	 */
	interceptionCoord=ballLine.getPerpendicularInterceptionPoint( Point2D(2.5, -3.0), valid);

	/*
	 * Verification
	 */
	EXPECT_FLOAT_EQ( 0.0f, interceptionCoord.x);
	EXPECT_FLOAT_EQ( -3.0f, interceptionCoord.y);
	EXPECT_TRUE( valid );
}


// MAIN
int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
