// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * adaptersCollector.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/adaptersCollector.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"


adaptersCollector::adaptersCollector()
/*!
 * \brief Adapter for receiving new worldmodel information from the outside
 * This class can be extended with update functions from different templated subject classes
 * That way we make a separation between ROS (or other future sources of updates)
 */
{
    _ballAdmin = NULL;
    _obstacleAdmin = NULL;
    _robotAdmin = NULL;
    _ballIsCaughtByBallHandlers = false;
    _rtdb = NULL;
    _robotStatus = robotStatusType::INVALID;
}

adaptersCollector::~adaptersCollector()
/*
 * The reason the Holy Grail has never been recovered is because nobody is
 * brave enough to ask Chuck Norris to give up his favourite coffee mug.
 */
{

}

void adaptersCollector::initializeRTDB()
{
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
}

void adaptersCollector::setBallAdministrator(ballAdministrator *ballAdmin)
{
    _ballAdmin = ballAdmin;
}

void adaptersCollector::setObstacleAdministrator(obstacleAdministrator *obstacleAdmin)
{
    _obstacleAdmin = obstacleAdmin;
}

void adaptersCollector::setRobotAdministrator(robotAdministrator *robotAdmin)
{
    _robotAdmin = robotAdmin;
}

void adaptersCollector::setRTDBInputAdapter(RTDBInputAdapter *rtdbInputAdapter)
{
    _rtdbInputAdapter = rtdbInputAdapter;
}

void adaptersCollector::setRTDBOutputAdapter(RTDBOutputAdapter *rtdbOutputAdapter)
{
    _rtdbOutputAdapter = rtdbOutputAdapter;
}

diagWorldModel adaptersCollector::getDiagnostics()
{
    return _diagnostics;
}

void adaptersCollector::reportToStdout()
{
    // get required data
    // TODO redesign class, make members
    robotClass_t robot = _robotAdmin->getLocalRobotPosition();
    bool ballPossession = _robotAdmin->getBallPossession();
    std::vector<ballClass_t> balls;
    _ballAdmin->getBalls(balls); // only valid balls, so without fakes
    std::vector<obstacleClass_t> obstacles;
    _obstacleAdmin->getObstacles(obstacles); // assuming fakes have been removed
    // ball may be missing
    char s[70] = {0};
    if (balls.size())
    {
        auto b = balls.at(0);
        sprintf(s, "bPos=[%6.2f %6.2f %6.2f] bVel=[%6.2f %6.2f %6.2f]", b.getX(), b.getY(), b.getZ(), b.getVX(), b.getVY(), b.getVZ());
    }
    tprintf("act=%d rPos=[%6.2f %6.2f %6.2f] rVel=[%6.2f %6.2f %6.2f] rConf=%.2f gotBall=%d #obst=%3d #ball=%d %s",
        _diagnostics.shared.inplay, robot.getX(), robot.getY(), robot.getTheta(), robot.getVX(), robot.getVY(), robot.getVTheta(), 
        _diagnostics.shared.visionConfidence, ballPossession, (int)obstacles.size(), (int)balls.size(), s);
}

void adaptersCollector::updateDiagnostics()
{
    if (_rtdb != NULL)
    {
        // visit administrators
        if (_robotAdmin != NULL)
        {
            _robotAdmin->fillDiagnostics(_diagnostics);
        }
        if (_ballAdmin != NULL)
        {
            _ballAdmin->fillDiagnostics(_diagnostics);
        }
        if (_obstacleAdmin != NULL)
        {
            _obstacleAdmin->fillDiagnostics(_diagnostics);
        }
        // some extra checks to generate warnings if needed
        bool ballIsCloseBy = _diagnostics.shared.ownBallsFirst && _diagnostics.local.balls.size();
        if (ballIsCloseBy)
        {
            float dataRate = _diagnostics.local.balls[0].ownGoodDataRate;
            float outliersFraction = _diagnostics.local.balls[0].outliersFraction;
            float ballTrackerAge = _diagnostics.local.balls[0].age;
            float freshness = _diagnostics.local.balls[0].freshness;
            bool fresh = freshness < 0.05;
            if (dataRate < 0.7) // this could be due to poor view on the ball
            {
                // but it could also be for other perfectly fine reasons - prevent false warnings
                // * if ball is just coming into view (low age)
                // * if ball is just going outside of view (high freshness)
                if (fresh && ballTrackerAge > 2.0)
                {
                    TRACE_WARNING_TIMEOUT(10.0, "ball data rate from vision too low (%.1f%%)", 100.0 * dataRate);
                }
            }
            if (outliersFraction > 0.5)
            {
                TRACE_WARNING_TIMEOUT(10.0, "too many ball outliers removed from ball result (%.1f%%)", 100.0 * outliersFraction);
            }
        }
        // write diagnostics data to RTDB
        _rtdb->put(DIAG_WORLDMODEL_LOCAL, &_diagnostics.local);
        _rtdb->put(DIAG_WORLDMODEL_SHARED, &_diagnostics.shared);
        // write an informative line to stdout using tprintf
        reportToStdout();
    }
}

bool checkBallCloseToObstacle(ballClass_t const &ball, std::vector<obstacleClass_t> const &obstacles)
{
    Point2D ballPos = Point2D(ball.getX(), ball.getY());
    // TODO: introduce a time stability factor, to maximize accuracy and consistency, required for teamplay decision making
    float proximityThreshold = 0.30; // TODO make configurable?
    for (auto itObst = obstacles.begin(); itObst != obstacles.end(); ++itObst)
    {
        Point2D obstaclePos = Point2D(itObst->getX(), itObst->getY());
        if (vectorsize(ballPos - obstaclePos) < proximityThreshold)
        {
            return true;
        }
    }
    return false;
}

void adaptersCollector::calcSelfBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot)
{
    TRACE_FUNCTION("");
    
    // initialize    
    bpType = ballPossessionTypeEnum::FIELD;
    bpRobot = 0;
    
    // step 1: own robot ball possession
    if (_robotAdmin->getBallPossession())
    {
        bpType = ballPossessionTypeEnum::TEAM;
        bpRobot = _myRobotId;
    }
    else
    {
        // step 2: check teammembers
        auto teamMembers = _robotAdmin->getTeammembers();
        for (auto it = teamMembers.begin(); it != teamMembers.end(); ++it)
        {
            if (it->getBallPossession())
            {
                bpType = ballPossessionTypeEnum::TEAM;
                bpRobot = it->getRobotID();

                Vector2D ball_pos_2d = getBallHandlerPosition(bpRobot);
                static const double BALL_NOMINAL_Z = 0.10;
                Vector3D ball_pos_3d(ball_pos_2d.x, ball_pos_2d.y, BALL_NOMINAL_Z); 
                _ballAdmin->appendBallPossessionMeasurements(ball_pos_3d, bpRobot, rtime::now());
                break;
            }
        }
    }
    TRACE("bpType=%s bpRobot=%d", enum2str(bpType), bpRobot);
}

void adaptersCollector::calcOpponentBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot)
{
    TRACE_FUNCTION("");
    // oponnent ballPossession is affected by multiple factors
    // we require that balls and obstacles are calculated FIRST

    if (bpType == ballPossessionTypeEnum::FIELD)
    {
        // step 3: check obstacles
        std::vector<ballClass_t> balls;
        _ballAdmin->getBalls(balls);
        if (balls.size())
        {
            // only consider first one in list
            auto ball = balls[0];
            std::vector<obstacleClass_t> obstacles;
            _obstacleAdmin->getObstacles(obstacles);
            if (checkBallCloseToObstacle(ball, obstacles))
            {
                bpType = ballPossessionTypeEnum::OPPONENT;
                bpRobot = 0;
            }
        }
    }
    TRACE("bpType=%s bpRobot=%d", enum2str(bpType), bpRobot);
}

Vector2D adaptersCollector::getBallHandlerPosition(int robotId)
{
    TRACE_FUNCTION("");
    #define BALL_HANDLER_RADIUS 0.25

    try
    {
        std::vector<robotClass_t> myTeam = _robotAdmin->getTeammembers();
        myTeam.push_back(_robotAdmin->getLocalRobotPosition()); // don't forget myself!
        for (auto it = myTeam.begin(); it != myTeam.end(); it++)
        {
            if (it->getRobotID() == robotId)
            {
                Position2D robotPos(it->getX(), it->getY(), it->getTheta());
                Position2D ballPosRcs(0, BALL_HANDLER_RADIUS, 0);
                Position2D ballPosFcs = ballPosRcs.transform_rcs2fcs(robotPos);
                return ballPosFcs.xy();
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    TRACE_ERROR("unreachable code");
    return Vector2D();
}

void adaptersCollector::ballPossessionOverrule(int bpRobot, rtime timeNow)
{
    TRACE_FUNCTION("");
    /*
    * Ball Possession to Ball Location overrule:
    * in case one of our robots has the ball firmly within its ball handlers, then
    * the ball location should NOT be calculated using ballTracking, because it is much less accurate
    * than the nominal ball position within ball handlers.
    */
    #define BALL_NOMINAL_Z 0.10
    Vector2D nominalBallPosition = getBallHandlerPosition(bpRobot);
    ballClass_t ball;
    ball.setCoordinates(nominalBallPosition.x, nominalBallPosition.y, BALL_NOMINAL_Z);
    ball.setVelocities(0.0, 0.0, 0.0);
    // zero velocity is not entirely accurate when robot with ball is moving (dribbling or rotating)
    // we could calculate it based on robot encoder feedback, but why would we --
    // no robot should respond anyway to this info (like, suddenly trying to intercept on this vector)
    ball.setConfidence(1.0);
    ball.setIsValid(true);
    ball.setId(0);
    ball.setTimestamp(timeNow); // TODO why do we keep timestamp in the ball ... no use case afaik
    // overrule
    _balls.clear();
    _balls.push_back(ball);
}

void adaptersCollector::heartBeatRecalculation(rtime const timeNow)
{
    TRACE_FUNCTION("");
    try
    {
        _diagnostics.shared.timestamp = timeNow;
        double t0 = rtime::now(); // for dutycycle tracing / diagnostics
        TRACE("> t=%16.6f", double(timeNow));

        if((_robotAdmin != NULL) &&
           (_ballAdmin != NULL) &&
           (_obstacleAdmin != NULL))
        {
            // Update configuration
            T_CONFIG_WORLDMODELSYNC config;
            _configAdapter.get(config);
            _rtdbInputAdapter->updateConfig(config);
            _rtdbOutputAdapter->updateConfig(config);

            // vision frame is required for the proccess functions
            _rtdbInputAdapter->getVisionFrame();

            // First get the localization measurements, and perform the localization tick
            // We do this first because the balls and obstacle measurements need the robot position for conversion to FCS
            _rtdbInputAdapter->processLocalizationCandidates();
            _rtdbInputAdapter->getInPlayState(timeNow);
            _robotAdmin->performCalculation(timeNow);

            // Then get all remaining data
            _rtdbInputAdapter->processBallCandidates();
            _rtdbInputAdapter->processObstacleCandidates();

            _rtdbInputAdapter->getVisionBallPossession();
            _rtdbInputAdapter->getRobotDisplacement(timeNow);
            _rtdbInputAdapter->getRobotVelocity(timeNow);
            _rtdbInputAdapter->getBallHandlingBallPossession();
            _rtdbInputAdapter->getTeamMembers();

            // when ball is in possession of our own team, that info is used
            // by ball tracking, so calculating self ball possession must be done
            // before calculating ball tracking
            ballPossessionTypeEnum bpType = ballPossessionTypeEnum::UNKNOWN;
            int bpRobot = 0;
            calcSelfBallPossession(bpType, bpRobot);

            // BallTracking -- BallAdministrator needs robot position for ownBallsFirst
            robotClass_t robot = _robotAdmin->getLocalRobotPosition(timeNow);
            Vector2D robotPos(robot.getX(), robot.getY());
            _ballAdmin->performCalculation(timeNow, robotPos);
            
            // ObstacleTracking -- teammembers are needed for fake obstacle removal (seeing own robots)
            _obstacleAdmin->notifyTeamMembers(_robotAdmin->getTeammembers());
            _obstacleAdmin->notifyOwnLocation(robot);
            _obstacleAdmin->performCalculation(timeNow);

            // Ball possession
            _ballAdmin->getBalls(_balls);
            // opponent ball possession depends on the balls positions and obstacle positions
            // so it must only be done after those calculations
            calcOpponentBallPossession(bpType, bpRobot);
            if (bpType == ballPossessionTypeEnum::TEAM)
            {
                ballPossessionOverrule(bpRobot, timeNow);
            }
            
            // send data to local and remote clients
            _rtdbOutputAdapter->setBallPossession(bpType, bpRobot);
            _rtdbOutputAdapter->setBalls(_balls);
            _rtdbOutputAdapter->setObstacles();
            _rtdbOutputAdapter->setRobotState(); // execution architecture: this one will trigger teamplay
            
            // gather and send diagnostics data
            _diagnostics.shared.duration = (double)(rtime::now() - t0);
            updateDiagnostics();

        }
        else
        {
            TRACE_ERROR("NULL pointer exception");
        }

        TRACE("<");
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

