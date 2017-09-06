 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testRobotLocalization.cpp
 *
 *  Created on: April 7th, 2017
 *      Author: Diana Koenraadt
 */

#include <gtest/gtest.h>

#include "int/algorithms/robotLocalization.hpp"
#include "int/configurators/localizationConfigurator.hpp"

#include "FalconsCommon.h"

std::string tstdir = "/home/robocup/falcons/code/packages/worldModel/tst/algorithms/";

/****************************
 selectBestCameraMeasurement
****************************/

TEST(robotLocalization, selectNearestCameraMeasurement_WhenCameraMeasurementEmpty_ShouldReturnFalse)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t measurement;
    float confidence;

    robotClass_t position;
    position.setCoordinates(0, 0, 0);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_FALSE(result);
}

TEST(robotLocalization, selectNearestCameraMeasurement_WhenMeasurementsZeroConfidence_ShouldReturnFalse)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setConfidence(0);
    robotMeasurementClass_t m2;
    m2.setConfidence(0);

    cameraMeasurements.push_back(m1);
    robotMeasurementClass_t measurement;
    float confidence;

    robotClass_t position;
    position.setCoordinates(0, 0, 0);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_FALSE(result);
}

TEST(robotLocalization, selectNearestCameraMeasurement_WhenMeasurementsSingleMeasurementPositiveConfidence_ShouldReturnThatMeasurement)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setPosition(1, 2, 3);
    m1.setConfidence(4);

    cameraMeasurements.push_back(m1);
    robotMeasurementClass_t measurement;
    float confidence;

    robotClass_t position;
    position.setCoordinates(0, 0, 0);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
    EXPECT_EQ(1, measurement.getX());
    EXPECT_EQ(2, measurement.getY());
    EXPECT_EQ(3, measurement.getTheta());
    EXPECT_EQ(4, measurement.getConfidence());
}

TEST(robotLocalization, selectNearestCameraMeasurement_WhenMeasurementsMultipleeMeasurements_ShouldReturnClosestMeasurement)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setPosition(4, 4, 4);
    m1.setConfidence(3);
    robotMeasurementClass_t m2;
    m2.setPosition(1, 1, 1);
    m2.setConfidence(3);

    cameraMeasurements.push_back(m1);
    cameraMeasurements.push_back(m2);
    robotMeasurementClass_t measurement;
    float confidence;

    robotClass_t position;
    position.setCoordinates(1, 1, 1);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
    EXPECT_EQ(1, measurement.getX());
    EXPECT_EQ(1, measurement.getY());
    EXPECT_EQ(1, measurement.getTheta());
    EXPECT_EQ(3, measurement.getConfidence());
}

TEST(robotLocalization, selectNearestCameraMeasurement_WhenMeasurementsDifferentTimestamp_ShouldReturnClosestTimestamp)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setPosition(1, 1, 1);
    m1.setConfidence(3);
    m1.setTimestamp(3);
    robotMeasurementClass_t m2;
    m2.setPosition(1, 1, 1);
    m1.setConfidence(4);
    m1.setTimestamp(10);

    cameraMeasurements.push_back(m1);
    cameraMeasurements.push_back(m2);
    robotMeasurementClass_t measurement;
    float confidence;
	
    robotClass_t position;
    position.setCoordinates(1, 1, 1);
    position.setTimestamp(9);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
    EXPECT_EQ(1, measurement.getX());
    EXPECT_EQ(1, measurement.getY());
    EXPECT_EQ(1, measurement.getTheta());
    EXPECT_EQ(4, measurement.getConfidence());
    EXPECT_EQ(10, measurement.getTimestamp());
}


TEST(robotLocalization, selectNearestCameraMeasurement_WhenMultiplePositionsInHistory_ShouldReturnClosestToPosition)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setPosition(1, 1, 1);
    m1.setConfidence(3);
    m1.setTimestamp(3);
    cameraMeasurements.push_back(m1);
    robotMeasurementClass_t m2;
    m2.setPosition(1, 2, 3);
    m2.setConfidence(4);
    m2.setTimestamp(10);
    cameraMeasurements.push_back(m2);

    robotMeasurementClass_t measurement;
    float confidence;

    robotClass_t position;
    position.setCoordinates(1, 1, 1);

    /*
     * Execution
     */
    bool result = robotLocalization::selectNearestCameraMeasurement(cameraMeasurements, position, measurement, confidence);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
    EXPECT_EQ(1, measurement.getX());
    EXPECT_EQ(1, measurement.getY());
    EXPECT_EQ(1, measurement.getTheta());
    EXPECT_EQ(3, measurement.getConfidence());
    EXPECT_EQ(3, measurement.getTimestamp());
}

/****************************
selectBestCameraMeasurement
****************************/

TEST(robotLocalization, selectBestCameraMeasurement_WhenCameraMeasurementEmpty_ShouldReturnFalse)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t measurement;

    /*
     * Execution
     */
    bool result = robotLocalization::selectBestCameraMeasurement(cameraMeasurements, measurement);

    /*
     * Verification
     */
    EXPECT_FALSE(result);
}

TEST(robotLocalization, selectBestCameraMeasurement_WhenCameraMeasurementSingle_ShouldReturnThatMeasurement)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setConfidence(1);
    cameraMeasurements.push_back(m1);

    robotMeasurementClass_t measurement;

    /*
     * Execution
     */
    bool result = robotLocalization::selectBestCameraMeasurement(cameraMeasurements, measurement);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
}

TEST(robotLocalization, selectBestCameraMeasurement_WhenCameraMeasurementSingle_ShouldReturnMeasurementWithHighestConfidence)
{
    /*
     * Setup
     */
    std::vector<robotMeasurementClass_t> cameraMeasurements;
    robotMeasurementClass_t m1;
    m1.setConfidence(1);
    cameraMeasurements.push_back(m1);

    robotMeasurementClass_t m2;
    m2.setConfidence(3);
    cameraMeasurements.push_back(m2);

    robotMeasurementClass_t measurement;

    /*
     * Execution
     */
    bool result = robotLocalization::selectBestCameraMeasurement(cameraMeasurements, measurement);

    /*
     * Verification
     */
    EXPECT_TRUE(result);
    EXPECT_EQ(3, measurement.getConfidence());
}

/****************************
 flipOrientation
****************************/

TEST(robotLocalization, flipOrientation_ShouldMirror180)
{
    /*
     * Setup
     */
    robotClass_t posVel;
    posVel.setCoordinates(1, 2, 0);

    /*
     * Execution
     */
    robotLocalization::flipOrientation(posVel);

    /*
     * Verification
     */
    EXPECT_EQ(-1, posVel.getX());
    EXPECT_EQ(-2, posVel.getY());
    EXPECT_NEAR(M_PI, posVel.getTheta(), 0.00001);
}

/****************************
 calculateWeightedPositionDelta
****************************/

TEST(robotLocalization, calculateWeightedPositionDelta_WhenLocationsIdentical_ShouldReturnDeltaZeroes)
{
    /*
     * Setup
     */
    Position2D currentPosition(5, 5, 5);
    Position2D newPosition(5, 5, 5);
    float weightFactor = 0.5;
    float dx, dy, dphi;

    /*
     * Execution
     */
    robotLocalization::calculateWeightedPositionDelta(currentPosition, newPosition, weightFactor, dx, dy, dphi);

    /*
     * Verification
     */
    EXPECT_EQ(0, dx);
    EXPECT_EQ(0, dy);
    EXPECT_EQ(0, dphi);
}

TEST(robotLocalization, calculateWeightedPositionDelta_WhenNewPositionLarger_ShouldReturnDeltaHalfTheDifference)
{
    /*
     * Setup
     */
    Position2D currentPosition(5, 5, 0.5);
    Position2D newPosition(10, 10, 1.0);
    float weightFactor = 0.5;
    float dx, dy, dphi;

    /*
     * Execution
     */
    robotLocalization::calculateWeightedPositionDelta(currentPosition, newPosition, weightFactor, dx, dy, dphi);

    /*
     * Verification
     */
    EXPECT_EQ(2.5, dx);
    EXPECT_EQ(2.5, dy);
    EXPECT_EQ(0.25, dphi);
}

TEST(robotLocalization, calculateWeightedPositionDelta_WhenNewPositionNegative_ShouldReturnDeltaHalfTheDifference)
{
    /*
     * Setup
     */
    Position2D currentPosition(0, 0, 0);
    Position2D newPosition(-10, -20, -M_PI / 2);
    float weightFactor = 1.0;
    float dx, dy, dphi;

    /*
     * Execution
     */
    robotLocalization::calculateWeightedPositionDelta(currentPosition, newPosition, weightFactor, dx, dy, dphi);

    /*
     * Verification
     */
    EXPECT_EQ(-10, dx);
    EXPECT_EQ(-20, dy);
    EXPECT_NEAR(project_angle_mpi_pi(-M_PI / 2), project_angle_mpi_pi(dphi), 0.00001);
}

/****************************
selectAnchor
****************************/

TEST(robotLocalization, selectAnchor_WhenMultipleElements_ShouldReturnClosestInTime)
{
    /*
     * Setup
     */
    std::vector<robotClass_t> posVelHistory;
    robotClass_t p1;
    p1.setCoordinates(1, 1, 1);
    p1.setTimestamp(100);
    robotClass_t p2;
    p2.setCoordinates(2, 2, 2);
    p2.setTimestamp(200);

    posVelHistory.push_back(p1);
    posVelHistory.push_back(p2);

    /*
     * Execution
     */
    int result = robotLocalization::selectAnchor(posVelHistory, 151);

    /*
     * Verification
     */
    EXPECT_EQ(result, 1);    
}

/****************************
 integration tests for vision weight factor
****************************/

TEST(robotLocalization, integrationTest_WhenVisionLatencyZero_ShouldApplyMotorDisplacementsOnly)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 0.0);

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0);
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(-10, -20, -M_PI);
    cameraMeasurement2.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 0 so should not be applied

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(5, 10, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(0);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be fully applied

    localizationAlgorithm.calculatePositionAndVelocity(1);

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();
    EXPECT_EQ(5, result.getX());
    EXPECT_EQ(10, result.getY());
    EXPECT_NEAR(project_angle_mpi_pi(M_PI / 2), project_angle_mpi_pi(result.getTheta()), 0.00001);
}

TEST(robotLocalization, integrationTest_WhenVisionLatencyOne_ShouldApplyMotorDisplacementAndFullCameraMeasurements)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 1.0);

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0);

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(5, 10, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(0);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be applied, every new displacement is taken into account
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(-10, -20, -M_PI);
    cameraMeasurement2.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 1 so should be fully applied on top of motorDisplacement

    localizationAlgorithm.calculatePositionAndVelocity(1);

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();
    EXPECT_EQ(-5, result.getX());
    EXPECT_EQ(-10, result.getY());
    EXPECT_NEAR(project_angle_mpi_pi(-M_PI / 2), project_angle_mpi_pi(result.getTheta()), 0.00001); 
}

TEST(robotLocalization, integrationTest_WhenVisionLatencyHalf_ShouldApplyMotorDisplacementAndHalfCameraMeasurement)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 0.5);

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0);

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(0);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be applied, every new displacement is taken into account
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(2, 4, M_PI / 2);
    cameraMeasurement2.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 0.5 so should be applied with weight factor 0.5 on top of (0,0,0). 

    localizationAlgorithm.calculatePositionAndVelocity(1);

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();
    EXPECT_EQ(36, result.getX());
    EXPECT_EQ(15, result.getY());
    EXPECT_NEAR(project_angle_mpi_pi(M_PI * 3 / 4), project_angle_mpi_pi(result.getTheta()), 0.00001); 
    // Note: Internally, the vision measurement is processed as a delta on top of (0,0,0), because it is nearest to the first vision lock coordinate (and the timestamps are equal and all the tweeaking parameters are zero...).
}

/****************************
 integration tests for vision latency
****************************/

TEST(robotLocalization, integrationTest_WhenVisionLatencyHalf_ShouldApplyCameraMeasurementOnClosestTimestamp)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 0.5); 
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionLatency, 1000); // Simulate fuckton of latency on vision	

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0);

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(1000);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be applied, every new displacement is taken into account
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(2, 4, M_PI / 2);
    cameraMeasurement2.setTimestamp(2000); // Thinks its from timestamp 2000, but we know how much latency there is: the timestamp is actually 1000
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 0.5 so should be applied with weight factor 0.5 on top of (0,0,0). 

    localizationAlgorithm.calculatePositionAndVelocity(2000);

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();
    EXPECT_EQ(35 - (35-2)/2.0, result.getX());
    EXPECT_EQ(13 - (13-4)/2.0, result.getY());
    EXPECT_NEAR(project_angle_mpi_pi(M_PI / 2), project_angle_mpi_pi(result.getTheta()), 0.00001); 
    // Note: Internally, the vision measurement is processed as a delta wrt to (35, 13, pi/2), because it is nearest to the first vision lock coordinate in terms of TIME
}

TEST(robotLocalization, integrationTest_WhenMotorDisplacementComesIn_ShouldBeAppliedOnTopOfPosition)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 0.5); 
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionLatency, 1000); // Simulate fuckton of latency on vision	

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0); // (A): results in (0,0,0)

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(1000);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be applied, every new displacement is taken into account
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(2, 4, M_PI / 2);
    cameraMeasurement2.setTimestamp(2000); // Thinks its from timestamp 2000, but we know how much latency there is: the timestamp is actually 1000
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 0.5 so should be applied with weight factor 0.5 on top of (0,0,0). 

    localizationAlgorithm.calculatePositionAndVelocity(1001); // (A): results in (18.5, 8.5, 0)

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();

    EXPECT_EQ(35 - (35-2)/2.0, result.getX());
    EXPECT_EQ(13 - (13-4)/2.0, result.getY());
    EXPECT_EQ(1001, result.getTimestamp());
    EXPECT_NEAR(project_angle_0_2pi(M_PI / 2), project_angle_0_2pi(result.getTheta()), 0.00001); 

    /*
     * Execution
     */
    robotDisplacementClass_t motorDisplacement2;
    motorDisplacement2.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement2.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement2.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement2.setTimestamp(3000);
    localizationAlgorithm.addDisplacement(motorDisplacement2); // Should be applied again, on top of (A), giving new position (B)
    
    localizationAlgorithm.calculatePositionAndVelocity(3001);

    /*
     * Verification
     */
    result = localizationAlgorithm.getRobotPositionAndVelocity();

    EXPECT_EQ(18.5 + 35, result.getX());
    EXPECT_EQ(8.5 + 13, result.getY());
    EXPECT_EQ(3001, result.getTimestamp());
    EXPECT_NEAR(project_angle_0_2pi(M_PI), project_angle_0_2pi(result.getTheta()), 0.00001); 
}

TEST(robotLocalization, integrationTest_WhenCameraMeasurementHistorical_ShouldUpdateHistoryAndExtrapolate)
{
    /*
     * Setup
     */
    robotLocalization localizationAlgorithm;
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, 0.5); 
    localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionLatency, 1000); // Simulate fuckton of latency on vision	

    /*
     * Execution
     */
    robotMeasurementClass_t cameraMeasurement1;
    cameraMeasurement1.setConfidence(1);
    cameraMeasurement1.setPosition(0, 0, 0);
    cameraMeasurement1.setTimestamp(0);
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement1); // Start position

    localizationAlgorithm.calculatePositionAndVelocity(0); // (A): results in (0,0,0)

    robotDisplacementClass_t motorDisplacement;
    motorDisplacement.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement.setTimestamp(1000);
    localizationAlgorithm.addDisplacement(motorDisplacement); // Should be applied, every new displacement is taken into account
    
    robotMeasurementClass_t cameraMeasurement2;
    cameraMeasurement2.setConfidence(1);
    cameraMeasurement2.setPosition(2, 4, M_PI / 2);
    cameraMeasurement2.setTimestamp(2000); // Thinks its from timestamp 2000, but we know how much latency there is: the timestamp is actually 1000
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement2); // Vision weight factor 0.5 so should be applied with weight factor 0.5 on top of (0,0,0). 

    localizationAlgorithm.calculatePositionAndVelocity(1001); // (A): results in (18.5, 8.5, 0)

    /*
     * Verification
     */
    robotClass_t result = localizationAlgorithm.getRobotPositionAndVelocity();
    EXPECT_EQ(18.5, result.getX());
    EXPECT_EQ(8.5, result.getY());
    EXPECT_EQ(1001, result.getTimestamp());
    EXPECT_NEAR(project_angle_0_2pi(M_PI / 2), project_angle_0_2pi(result.getTheta()), 0.00001); 

    /*
     * Execution
     */
    robotDisplacementClass_t motorDisplacement2;
    motorDisplacement2.setCoordinateType(coordinateType::FIELD_COORDS);
    motorDisplacement2.setDeltaPosition(35, 13, M_PI / 2);
    motorDisplacement2.setDisplacementSource(displacementType::MOTORS);
    motorDisplacement2.setTimestamp(3000);
    localizationAlgorithm.addDisplacement(motorDisplacement2); // Should be applied again, on top of (A), giving new position (B)
    
    robotMeasurementClass_t cameraMeasurement3;
    cameraMeasurement3.setConfidence(1);
    cameraMeasurement3.setPosition(2, 4, M_PI / 2);
    cameraMeasurement3.setTimestamp(2000); // Thinks its from timestamp 2000, but we know how much latency there is: the timestamp is actually 1000	
    localizationAlgorithm.addVisionMeasurement(cameraMeasurement3); // Vision weight factor 0.5 so should be applied with weight factor 0.5 on top of (18.5, 8.5, 0).

    localizationAlgorithm.calculatePositionAndVelocity(3001);

    /*
     * Verification
     */
    result = localizationAlgorithm.getRobotPositionAndVelocity();

    EXPECT_EQ(18.5 - (18.5-2)/2.0 + 35, result.getX());
    EXPECT_EQ(8.5 - (8.5-4)/2.0 + 13, result.getY());
    EXPECT_EQ(3001, result.getTimestamp());
    EXPECT_NEAR(project_angle_0_2pi(M_PI), project_angle_0_2pi(result.getTheta()), 0.00001); 
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
