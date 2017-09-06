 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testLocalization.cpp
 *
 *  Created on: Oct 09, 2016
 *      Author: Jan Feitsma
 */

#include <gtest/gtest.h>

#include "int/algorithms/robotLocalization.hpp"
#include "int/configurators/localizationConfigurator.hpp"

#include "FalconsCommon.h"


// test that sensible data is produced before any displacement/vision input has arrived 
TEST(testLocalization, empty)
{
    TRACE("test empty start");
    
    // setup
    robotLocalization loc;

    // execution: don't feed any data, only do a GET
    loc.calculatePositionAndVelocity(0);
    robotClass_t posvel = loc.getRobotPositionAndVelocity();

    // verification
    EXPECT_FLOAT_EQ(0.0, posvel.getX());
    EXPECT_FLOAT_EQ(0.0, posvel.getY());
    EXPECT_FLOAT_EQ(0.0, posvel.getTheta());
    EXPECT_FLOAT_EQ(0.0, posvel.getVX());
    EXPECT_FLOAT_EQ(0.0, posvel.getVY());
    EXPECT_FLOAT_EQ(0.0, posvel.getVTheta());
    
    TRACE("test empty end");
}

// check that displacement values are taken as-is cumulatively
// and that the array is reset after each get tick
TEST(testLocalization, displacementOnlyFcs)
{
    TRACE("test displacementOnlyFcs start");
    
    // setup: give a zero vision measurement to have encoder displacement accepted
    robotLocalization loc;
    robotMeasurementClass_t vis;
    vis.setConfidence(1.0);
    vis.setPosition(0.0, 0.0, 0.0);
    loc.addVisionMeasurement(vis);
    loc.calculatePositionAndVelocity(0);

    // execution: feed a few displacement measurements (FCS) and query the solver twice
    robotDisplacementClass_t displ;
    displ.setCoordinateType(coordinateType::FIELD_COORDS);
    displ.setDisplacementSource(displacementType::MOTORS);
    displ.setDeltaPosition(1.0, -1.0, 0.0);
    loc.addDisplacement(displ);
    displ.setDeltaPosition(1.0, 0.5, 0.3);
    loc.addDisplacement(displ);
    loc.calculatePositionAndVelocity(10.0);
    robotClass_t posvel1 = loc.getRobotPositionAndVelocity();
    displ.setDeltaPosition(-1.0, 1.0, 0.5);
    loc.addDisplacement(displ);
    displ.setDeltaPosition(-1.0, 0.2, -0.2);
    loc.addDisplacement(displ);
    // query the solver
    loc.calculatePositionAndVelocity(20.0);
    robotClass_t posvel2 = loc.getRobotPositionAndVelocity();

    // verification
    EXPECT_FLOAT_EQ(2.0, posvel1.getX());
    EXPECT_FLOAT_EQ(-0.5, posvel1.getY());
    EXPECT_FLOAT_EQ(0.3, posvel1.getTheta());
    EXPECT_FLOAT_EQ(0.0, posvel2.getX());
    EXPECT_FLOAT_EQ(0.7, posvel2.getY());
    EXPECT_FLOAT_EQ(0.6, posvel2.getTheta());

    TRACE("test displacementOnlyFcs end");
}

// test rcs2fcs transformation 
TEST(testLocalization, displacementOnlyRcs)
{
    TRACE("test displacementOnlyRcs start");

    // setup: give a zero vision measurement to have encoder displacement accepted
    robotLocalization loc;
    robotMeasurementClass_t vis;
    vis.setConfidence(1.0);
    vis.setPosition(0.0, 0.0, 0.0);
    loc.addVisionMeasurement(vis);
    loc.calculatePositionAndVelocity(0);

    // execution: feed a few displacement measurements (RCS)
    robotDisplacementClass_t displ;
    displ.setCoordinateType(coordinateType::ROBOT_COORDS);
    displ.setDisplacementSource(displacementType::MOTORS);
    displ.setDeltaPosition(1.0, 2.0, 0.0);
    loc.addDisplacement(displ);
    displ.setDeltaPosition(0.0, 0.0, M_PI*0.25);
    loc.addDisplacement(displ);
    displ.setDeltaPosition(0.0, -sqrt(2.0), 0.0);
    loc.addDisplacement(displ);
    // query the solver
    loc.calculatePositionAndVelocity(10.0);
    robotClass_t posvel = loc.getRobotPositionAndVelocity();

    // verification
    EXPECT_FLOAT_EQ(1.0, posvel.getX());
    EXPECT_FLOAT_EQ(-2.0, posvel.getY());
    EXPECT_FLOAT_EQ(M_PI*0.25, posvel.getTheta());
    
    TRACE("test displacementOnlyRcs end");
}

// test that first vision frame is taken over as-is
TEST(testLocalization, visionFirstLoc)
{
    TRACE("test visionFirstLoc start");

    // setup
    robotLocalization loc;

    // execution
    // feed a single vision candidate but with too low confidence
    robotMeasurementClass_t vis;
    vis.setConfidence(0.0);
    vis.setPosition(0.5, -0.7, 2.9);
    loc.addVisionMeasurement(vis);
    // query the solver
    loc.calculatePositionAndVelocity(0);
    robotClass_t posvel1 = loc.getRobotPositionAndVelocity();
    // feed the same vision candidate but with good confidence
    vis.setConfidence(1.0);
    vis.setPosition(0.5, -0.7, 2.9);
    loc.addVisionMeasurement(vis);
    // query the solver
    loc.calculatePositionAndVelocity(1.0);
    robotClass_t posvel2 = loc.getRobotPositionAndVelocity();

    // verification
    EXPECT_FLOAT_EQ(0.0, posvel1.getX());
    EXPECT_FLOAT_EQ(0.0, posvel1.getY());
    EXPECT_FLOAT_EQ(0.0, posvel1.getTheta());
    EXPECT_FLOAT_EQ(0.5, posvel2.getX());
    EXPECT_FLOAT_EQ(-0.7, posvel2.getY());
    EXPECT_FLOAT_EQ(2.9, posvel2.getTheta());
    
    TRACE("test visionFirstLoc end");
}

// test that second vision frame is taken with weight factor
TEST(testLocalization, visionConverge)
{
    TRACE("test visionConverge start");

    // setup
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::trackerScoreAcceptanceThreshold, 1.5);
    robotLocalization loc;

    // execution: feed a single vision candidate
    robotMeasurementClass_t vis;
    vis.setConfidence(1.0);
    vis.setPosition(0.2, 0.4, 0.6);
    loc.addVisionMeasurement(vis);
    // query the solver for the first time to make it lock, don't use the result
    loc.calculatePositionAndVelocity(0);
    // feed second vision candidate
    vis.setPosition(0.0, 0.0, 0.0);
    loc.addVisionMeasurement(vis);
    // query the solver for the second time
    loc.calculatePositionAndVelocity(1.0);
    robotClass_t posvel = loc.getRobotPositionAndVelocity();

    // verification: the second candidate is used for only 20%
    EXPECT_FLOAT_EQ(0.16, posvel.getX());
    EXPECT_FLOAT_EQ(0.32, posvel.getY());
    EXPECT_FLOAT_EQ(0.48, posvel.getTheta());
    
    TRACE("test visionConverge end");
}

// test that the best fit of a sequence of vision candidates is chosen
TEST(testLocalization, visionSelect)
{
    TRACE("test visionSelect start");

    // setup
    robotLocalization loc;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::trackerScoreAcceptanceThreshold, 1.5);

    // execution
    // feed a single vision candidate
    robotMeasurementClass_t vis;
    vis.setConfidence(1.0);
    vis.setPosition(-3, 5, 2);
    loc.addVisionMeasurement(vis);
    // query the solver for the first time to make it lock, don't use the result
    loc.calculatePositionAndVelocity(0);

    // feed a list of vision candidates
    vis.setPosition(0.0, 0.0, 0.0); // few meters away --> reject
    loc.addVisionMeasurement(vis);
    vis.setPosition(3, -5, 2); // mirrored position --> reject
    loc.addVisionMeasurement(vis);
    vis.setPosition(-3, 5, 2+M_PI); // same position but halfway rotated --> reject
    loc.addVisionMeasurement(vis);
    vis.setPosition(-3, 4, 2.1); // only a single meter away --> accept
    loc.addVisionMeasurement(vis);
    vis.setPosition(2, 2, 1); // another one which is a few meters away --> reject
    loc.addVisionMeasurement(vis);

    // query the solver for the second time
    loc.calculatePositionAndVelocity(0.1);
    robotClass_t posvel = loc.getRobotPositionAndVelocity();

    // verification: the best candidate is used (but for only 20%)
    EXPECT_FLOAT_EQ(-3, posvel.getX());
    EXPECT_FLOAT_EQ(4.8, posvel.getY());
    EXPECT_FLOAT_EQ(2.02, posvel.getTheta());
    
    TRACE("test visionSelect end");
}

// test that initial orientation is chosen such that robot will play forward (+y)
TEST(testLocalization, flipOrientation)
{
    TRACE("test flipOrientation start");

    // setup
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::trackerScoreAcceptanceThreshold, 6.0);
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 1.0);
    robotLocalization loc;

    // execution: feed a single vision candidate
    robotMeasurementClass_t vis;
    vis.setConfidence(1.0);
    float y = 0.3; // must be a bit small, otherwise we get vision glitch and speed warnings
    vis.setPosition(0, y, 5); // looking at -y
    loc.addVisionMeasurement(vis);
    // query the solver for the first time to make it lock, which implicitly will flip orientation
    loc.calculatePositionAndVelocity(0);
    robotClass_t posvel1 = loc.getRobotPositionAndVelocity();
    // rotate a bit by feed another vision measurement
    vis.setPosition(0, -y, 3.5); 
    loc.addVisionMeasurement(vis);
    loc.calculatePositionAndVelocity(1);
    robotClass_t posvel2 = loc.getRobotPositionAndVelocity();
    // finally, re-initialize to make it play forward again
    loc.triggerInitialization(0);
    loc.addVisionMeasurement(vis);
    loc.calculatePositionAndVelocity(10);
    robotClass_t posvel3 = loc.getRobotPositionAndVelocity();

    // verification
    float expectedY = -y;
    float expectedTheta = 3.5;
    EXPECT_FLOAT_EQ(0, posvel1.getX());
    EXPECT_FLOAT_EQ(-y, posvel1.getY());
    EXPECT_FLOAT_EQ(5-M_PI, posvel1.getTheta());
    EXPECT_FLOAT_EQ(0, posvel2.getX());
    EXPECT_FLOAT_EQ(expectedY, posvel2.getY());
    EXPECT_FLOAT_EQ(expectedTheta, posvel2.getTheta());
    EXPECT_FLOAT_EQ(0, posvel3.getX());
    EXPECT_FLOAT_EQ(-expectedY, posvel3.getY());
    EXPECT_FLOAT_EQ(expectedTheta-M_PI, posvel3.getTheta());
    
    TRACE("test flipOrientation end");
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
