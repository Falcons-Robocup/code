 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
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

void adaptersCollector::calcBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot)
{
    // ballPossession is affected by multiple factors
    // we require that balls and obstacles are calculated FIRST
    // also, we will inspect other robot 'hasball' claims

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
            }
        }
        
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
    }
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

            // First get the localization measurements, and perform the localization tick
            // We do this first because the balls and obstacle measurements need the robot position for conversion to FCS
            _rtdbInputAdapter->getLocalizationCandidates();
            _rtdbInputAdapter->getInPlayState();
            _robotAdmin->performCalculation(timeNow);

            // Then get all remaining data
            _rtdbInputAdapter->getBallCandidates();
            _rtdbInputAdapter->getObstacleCandidates();
            _rtdbInputAdapter->getVisionBallPossession();
            _rtdbInputAdapter->getRobotDisplacement();
            _rtdbInputAdapter->getRobotVelocity();
            _rtdbInputAdapter->getBallHandlingBallPossession();
            _rtdbInputAdapter->getTeamMembers();

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
            ballPossessionTypeEnum bpType = ballPossessionTypeEnum::UNKNOWN;
            int bpRobot = 0;
            calcBallPossession(bpType, bpRobot);
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

