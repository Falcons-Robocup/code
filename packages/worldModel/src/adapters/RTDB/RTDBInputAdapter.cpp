 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */


#include "int/adapters/RTDB/RTDBInputAdapter.hpp"
//
#include "falconsCommon.hpp" //getRobotNumber()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

RTDBInputAdapter::RTDBInputAdapter()
{
    TRACE_FUNCTION("");
    // init data
    _visionBallPossession = false;
    _inPlay = robotStatusType::INVALID;
    _ballAdmin = 0;
    _robotAdmin = 0;
    _obstacleAdmin = 0;
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    _robotDisplacementInitialized = false;

    _uIDGenerator = identifierGenerator(_myRobotId);
}

RTDBInputAdapter::~RTDBInputAdapter()
{
}

void RTDBInputAdapter::setBallAdministrator(ballAdministrator *ballAdmin)
{
    _ballAdmin = ballAdmin;
}

void RTDBInputAdapter::setObstacleAdministrator(obstacleAdministrator *obstacleAdmin)
{
    _obstacleAdmin = obstacleAdmin;
}

void RTDBInputAdapter::setRobotAdministrator(robotAdministrator *robotAdmin)
{
    _robotAdmin = robotAdmin;
}

void RTDBInputAdapter::updateConfig(T_CONFIG_WORLDMODELSYNC const &config)
{
    _config = config;
}

template <typename T1>
T1 convertObjectRCSToFCS(T1 &objCandidate, Position2D &robotPos)
{
    float camZ = objCandidate.cameraZ;
    Position2D camRcs(objCandidate.cameraX, objCandidate.cameraY, objCandidate.cameraPhi);
    Position2D camFcs = camRcs.transform_rcs2fcs(robotPos);

    T1 measurementFCS = objCandidate;
    measurementFCS.cameraX = camFcs.x;
    measurementFCS.cameraY = camFcs.y;
    measurementFCS.cameraZ = camZ;
    measurementFCS.cameraPhi = camFcs.phi;

    return measurementFCS;
}

bool RTDBInputAdapter::useVisionFromRobot(int robotId)
{
    // get teamId belonging to given robotId
    T_ROBOT_STATE robotState;
    int r = _rtdbClient._rtdb.at(robotId)->get(ROBOT_STATE, &robotState, robotId);
    if (r != RTDB2_SUCCESS)
    {
        return false;
    }
    teamIdType otherTeamId = robotState.teamId;
    if (_config.useVisionFrom.find(otherTeamId) != std::string::npos)
    {
        // id found
        return true;
    }
    return false;
}

void RTDBInputAdapter::getBallCandidates()
{
    TRACE_FUNCTION("");
    // only continue if valid loc
    if (!_robotAdmin->inplay())
    {
        return;
    }

    // First put the Local FCS data to RTDB
    T_BALL_CANDIDATES_FCS measurementsFcs;
    int r = _rtdb->get(BALL_CANDIDATES, &_ballCandidates);
    if (r == RTDB2_SUCCESS)
    {
        // convert camera position from RCS to FCS for each ball measurement
        if (_ballCandidates.size())
        {
            robotClass_t pos = _robotAdmin->getLocalRobotPosition(_ballCandidates[0].timestamp);
            Position2D robotPos(pos.getX(), pos.getY(), pos.getTheta());

            for (auto it = _ballCandidates.begin(); it != _ballCandidates.end(); ++it)
            {
                ballMeasurement ballMeasFcs = convertObjectRCSToFCS(*it, robotPos);
                measurementsFcs.push_back(ballMeasFcs);
            }

        }
        if (_config.shareVision)
        {
            _rtdb->put(BALL_CANDIDATES_FCS, &measurementsFcs);
        }
    }

    // Then get the Shared FCS data from RTDB
    std::vector<int>::const_iterator it;
    for (it = _rtdbClient._agentIds.begin(); it != _rtdbClient._agentIds.end(); ++it)
    {
        if (*it != _myRobotId)
        {
            // Skip if configured to ignore
            if (!useVisionFromRobot(*it))
            {
                continue;
            }

            T_BALL_CANDIDATES_FCS sharedMeasurements;
            r = _rtdbClient._rtdb.at(*it)->get(BALL_CANDIDATES_FCS, &sharedMeasurements, *it);

            if (r == RTDB2_SUCCESS)
            {
                // Add sharedMeasurements to measurementsFcs
                measurementsFcs.insert(measurementsFcs.end(), sharedMeasurements.begin(), sharedMeasurements.end());
            }
        }
    }

    // Finally, write measurements to the administrator
    _ballAdmin->appendBallMeasurements(measurementsFcs);
}

void RTDBInputAdapter::getObstacleCandidates()
{
    TRACE_FUNCTION("");
    // only continue if valid loc
    if (!_robotAdmin->inplay())
    {
        return;
    }

    int r = _rtdb->get(OBSTACLE_CANDIDATES, &_obstacleCandidates);

    // First put the Local FCS candidates to RTDB for sharing with other robots
    T_OBSTACLE_CANDIDATES_FCS measurementsFcs;
    if (r == RTDB2_SUCCESS)
    {
        // convert camera position from RCS to FCS for each obstacle measurement
        // while performing latency correction, assuming all obstacles have the same timestamp (TODO)
        if (_obstacleCandidates.size())
        {
            robotClass_t pos = _robotAdmin->getLocalRobotPosition(_obstacleCandidates[0].timestamp);
            Position2D robotPos(pos.getX(), pos.getY(), pos.getTheta());
            int iter = 0;

            for (auto it = _obstacleCandidates.begin(); it != _obstacleCandidates.end(); ++it)
            {
                obstacleMeasurement obstacleMeasFcs = convertObjectRCSToFCS(*it, robotPos);
                measurementsFcs.push_back(obstacleMeasFcs);
                iter++;
            }

            if (_config.shareVision)
            {
                _rtdb->put(OBSTACLE_CANDIDATES_FCS, &measurementsFcs);
            }
        }
    }

    // Then get the Shared FCS candidates from RTDB from other robots
    std::vector<int>::const_iterator it;
    for (it = _rtdbClient._agentIds.begin(); it != _rtdbClient._agentIds.end(); ++it)
    {
        if (*it != _myRobotId)
        {
            // Skip if configured to ignore
            if (!useVisionFromRobot(*it))
            {
                continue;
            }

            T_OBSTACLE_CANDIDATES_FCS sharedMeasurements;
            r = _rtdbClient._rtdb.at(*it)->get(OBSTACLE_CANDIDATES_FCS, &sharedMeasurements, *it);

            if (r == RTDB2_SUCCESS)
            {
                // Add sharedMeasurements to measurementsFcs
                measurementsFcs.insert(measurementsFcs.end(), sharedMeasurements.begin(), sharedMeasurements.end());
            }
        }
    }

    // Finally, write measurements to the administrator
    _obstacleAdmin->appendObstacleMeasurements(measurementsFcs);
}

void RTDBInputAdapter::getLocalizationCandidates()
{
    TRACE_FUNCTION("");
    int r = _rtdb->get(LOCALIZATION_CANDIDATES, &_localizationCandidates, _myRobotId);
    if (r != RTDB2_SUCCESS)
    {
        return;
    }

    // convert sharedType to WM internal type
    std::vector<robotMeasurementClass_t> robotMeasurements;
    T_LOCALIZATION_CANDIDATES::const_iterator it;
    for (it = _localizationCandidates.begin(); it != _localizationCandidates.end(); ++it)
    {
        robotMeasurementClass_t robotMeas;
        robotMeas.setID(it->identifier);
        robotMeas.setTimestamp(it->timestamp);
        robotMeas.setConfidence(it->confidence);
        robotMeas.setCoordinateType(coordinateType::FIELD_COORDS);
        robotMeas.setPosition(it->x, it->y, it->theta);

        robotMeasurements.push_back(robotMeas);
    }

    if (r == RTDB2_SUCCESS)
    {
        // Write measurements to the administrator
        _robotAdmin->appendRobotVisionMeasurements(robotMeasurements);
    }
}
void RTDBInputAdapter::getVisionBallPossession()
{
    TRACE_FUNCTION("");
    RtDB2Item item;
    int r = _rtdb->getItem(VIS_BALL_POSSESSION, item);
    if (r != RTDB2_SUCCESS)
    {
        return;
    }

    // _visionBallPossession is only written when TRUE, so use age to determine ball possession
    // ball possession when age < 500ms
    _visionBallPossession = item.value<T_VIS_BALL_POSSESSION>();
    if (_visionBallPossession && item.age() < 0.500)
    {
        _robotAdmin->setBallPossessionVision(true);
    }
    else
    {
        _robotAdmin->setBallPossessionVision(false);
    }
}


void RTDBInputAdapter::getRobotDisplacement(rtime timeNow)
{
    TRACE_FUNCTION("");

    int ageMs = 0;

    T_ROBOT_DISPLACEMENT_FEEDBACK newDisplacement;

    int r = _rtdb->get(ROBOT_DISPLACEMENT_FEEDBACK, &newDisplacement, ageMs, _myRobotId);
    if (r != RTDB2_SUCCESS)
    {
        return;
    }

    if (_robotDisplacementInitialized)
    {

        std::vector<robotDisplacementClass_t> displacements;

        robotDisplacementClass_t displacement;
        displacement.setID(_uIDGenerator.getUniqueID());
        displacement.setTimestamp(rtime::now());
        displacement.setCoordinateType(coordinateType::ROBOT_COORDS);
        displacement.setDisplacementSource(displacementType::MOTORS);

        displacement.setDeltaPosition(
            newDisplacement.x - _prevRobotDisplacement.x,
            newDisplacement.y - _prevRobotDisplacement.y,
            newDisplacement.Rz - _prevRobotDisplacement.Rz);

        displacements.push_back(displacement);

        // tprintf("encoder displacement: dx=%7.3f dy=%7.3f dRz=%7.3f", displacement.getdX(), displacement.getdY(), displacement.getdTheta());

        if(_robotAdmin != NULL)
        {
            _robotAdmin->appendRobotDisplacementMeasurements(displacements);
        }
    }

    else
    {
        // The motors can have a big displacement when worldModel starts.
        // Therefore we use the first tick to initialize _prevRobotDisplacement with the displacements in RTDB.
        _robotDisplacementInitialized = true;
    }

    _prevRobotDisplacement = newDisplacement;
}

void RTDBInputAdapter::getRobotVelocity(rtime timeNow)
{
    TRACE_FUNCTION("");

    int r = _rtdb->get(ROBOT_VELOCITY_FEEDBACK, &_robotVelocity);
    if (r != RTDB2_SUCCESS)
    {
        return;
    }

    std::vector<robotVelocityClass_t> velocities;

    robotVelocityClass_t velocity;
    velocity.setID(_uIDGenerator.getUniqueID());
    velocity.setTimestamp(timeNow);
    velocity.setCoordinateType(coordinateType::ROBOT_COORDS);
    velocity.setDisplacementSource(displacementType::MOTORS);

    velocity.setDeltaVelocity(
            _robotVelocity.x,
            _robotVelocity.y,
            _robotVelocity.Rz);

    velocities.push_back(velocity);

    if(_robotAdmin != NULL)
    {
        _robotAdmin->appendRobotVelocityMeasurements(velocities);
    }

    //tprintf("encoder velocity    : dx=%7.3f dy=%7.3f dRz=%7.3f", _robotVelocity.x, _robotVelocity.y, _robotVelocity.Rz);
}

void RTDBInputAdapter::enableInplayOverrule()
{
    _inplayOverrule = true;
}

void RTDBInputAdapter::getInPlayState(rtime timeNow)
{
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK currentInPlay;

    if (_config.forceInplay == true || _inplayOverrule)
    {
        // testing override
        currentInPlay = robotStatusEnum::INPLAY;
    }
    else
    {
        // regular behavior: get data from RTDB
        int r = 0;
        r = _rtdb->get(INPLAY_FEEDBACK, &currentInPlay);
        // check if we have OK data
        if (r != RTDB2_SUCCESS)
        {
            return;
        }
    }

    // convert to worldmodel's inplay type
    robotStatusType wmInPlay = robotStatusType::INVALID;
    switch(currentInPlay)
    {
        case robotStatusEnum::INPLAY:
        {
            wmInPlay = robotStatusType::INPLAY;
            break;
        }

        case robotStatusEnum::OUTOFPLAY:
        {
            wmInPlay = robotStatusType::OUTOFPLAY;
            break;
        }

        default:
        {
            TRACE_ERROR("Received unexpected robot status type %d", currentInPlay);
            break;
        }
    }

    // new state detected
    if (_inPlay != wmInPlay)
    {
        _robotAdmin->setRobotStatus(_myRobotId, wmInPlay, timeNow);
        // generate an event, for eventlogging on coach
        if (wmInPlay == robotStatusType::INPLAY)
        {
            TRACE_INFO("switched to INPLAY");
        }
        else
        {
            TRACE_INFO("switched to OUTOFPLAY");
        }
        _inPlay = wmInPlay;
    }
}


void RTDBInputAdapter::getBallHandlingBallPossession()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_BALL_POSSESSION ballIsCaughtByBallHandlers;
    int r = _rtdb->get(BALLHANDLERS_BALL_POSSESSION, &ballIsCaughtByBallHandlers);
    if (r != RTDB2_SUCCESS)
    {
        return;
    }
    _robotAdmin->setBallPossessionBallHandlers(ballIsCaughtByBallHandlers);
}

void RTDBInputAdapter::getTeamMembers()
{
    TRACE_FUNCTION("");
    // check for NULL pointers
    if ((_robotAdmin == NULL) || (_rtdb == NULL))
    {
        TRACE_ERROR("NULL pointer exception");
        return;
    }
    // get data from teammembers
    T_ROBOT_STATE robotRtdb;
    for (int agent = 1; agent <= MAX_ROBOTS; ++agent)
    {
        // only process teammembers, not ourself
        if (agent != _myRobotId)
        {
            int r = _rtdb->get(ROBOT_STATE, &robotRtdb, agent);
            // require a successful GET but also data not too old (say from a previous session)
            if (r == RTDB2_SUCCESS)
            {
                // convert to wm-internal struct robotClass_t
                robotClass_t robot;
                robot.setRobotID(agent);
                robot.setTimestamp(robotRtdb.timestamp);
                robot.setCoordinateType(coordinateType::FIELD_COORDS); // unused, TODO remove
                robot.setCoordinates(robotRtdb.position.x, robotRtdb.position.y, robotRtdb.position.Rz);
                robot.setVelocities(robotRtdb.velocity.x, robotRtdb.velocity.y, robotRtdb.velocity.Rz);
                robot.setBallPossession(robotRtdb.hasBall);
                // determine inplay status
                robotStatusType status = robotStatusType::OUTOFPLAY;
                if (robotRtdb.status == robotStatusEnum::INPLAY)
                {
                    status = robotStatusType::INPLAY;
                }
                // feed to administrator
                _robotAdmin->updateRobotPositionAndVelocity(robot);
                _robotAdmin->setRobotStatus(robot.getRobotID(), status, robot.getTimestamp());
            }
        }
    }
}
