 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testBallDiscriminator.cpp
 *
 *  Created on: Sep 06, 2016
 *      Author: Jan Feitsma
 */

#include <gtest/gtest.h>

#include "int/administrators/ballDiscriminator.hpp"
#include "int/administrators/ballStimulator.hpp"
#include "int/configurators/ballTrackerConfigurator.hpp"
#include "timeConvert.hpp"

#include "FalconsCommon.h"

std::string tstdir = "/home/robocup/falcons/code/packages/worldModel/tst/ball/";

TEST(testBallDiscriminator, empty)
{
	/*
	 * Setup
	 */
	ballDiscriminator discriminator;

	/*
	 * Execution
	 */
	discriminator.performCalculation(0.0);
    std::vector<ballClass_t> balls = discriminator.getBalls();

	/*
	 * Verification
	 */
	EXPECT_TRUE(balls.size() == 0);
}

TEST(testBallDiscriminator, timeConvert)
{
    double t1 = 12345.678901;
    TRACE("t1=%16.8f", t1);
    struct timeval t2 = double2timeval(t1);
    TRACE("t2.sec=%ld", t2.tv_sec);
    TRACE("t2.usec=%ld", t2.tv_usec);
    double t3 = timeval2double(t2);
    TRACE("t3=%16.8f", t3);
    // check that the conversion timeval -> double -> timeval works within microsecond accuracy
	EXPECT_TRUE(fabs(t3 - t1) < 1e-6);
}

TEST(testBallDiscriminator, timeout)
{
	/*
	 * Setup
	 */
	ballDiscriminator discriminator;
	// disable confidence heuristics
	ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceGoodLimit, -1);

	/*
	 * Execution
	 */
	 
	// feed a ball measurement at t=0
    ballMeasurementType measurement;
    measurement.setTimestamp(0.0);
    measurement.setConfidence(1.0);
    measurement.setSphericalCoords(
    	0.0,
    	0.0,
    	1.0);
    measurement.setCameraOffset(
    	-2.0,
    	-2.0,
    	1.0,
    	0.0);
    TRACE("add1");
    discriminator.addMeasurement(measurement);
    
    // query the solver at t=0.5, expecting a single ball
    std::vector<ballClass_t> balls1;
    TRACE("get1");
    discriminator.performCalculation(0.5);
    balls1 = discriminator.getBalls();
    
    // query the solver at t=10.0, expecting no balls
    std::vector<ballClass_t> balls2;
    TRACE("get2");
    discriminator.performCalculation(10.0);
    balls2 = discriminator.getBalls();

	/*
	 * Verification
	 */
	EXPECT_TRUE(balls1.size() == 1);
	EXPECT_TRUE(balls2.size() == 0);
}

TEST(testBallDiscriminator, stimNoMeas)
{
	// Setup
    std::string inputFilename = tstdir + "testInput01.txt";
    
	// Execution
	ballStimulator b(inputFilename);

	// Verification
	EXPECT_TRUE(b.compare());
}

TEST(testBallDiscriminator, stimSpeed)
{
	// Setup
    std::string inputFilename = tstdir + "testInput02.txt";
    
	// Execution
	ballStimulator b(inputFilename);

	// Verification
	EXPECT_TRUE(b.compare());
}

TEST(testBallDiscriminator, stimTraceVdl20160929)
{
    // This test uses 15 seconds of real data, captured using frontCam of 3 robots, during VDL testmatch
    // We use this data to port and test tracker implementation, focusing on ball confidence heuristic.
    // Probably we also need to debug on a lower level (e.g. confidence breakdown, number of trackers per
    // solver tick), that is done by comparing tracing. 
    
	// Setup
    std::string inputFilename = tstdir + "testInput03.txt";
    
	// Execution
	ballStimulator b(inputFilename);

	// Verification
	EXPECT_TRUE(b.compare());
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
