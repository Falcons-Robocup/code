// Copyright 2018-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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


#define CAMERA_HEIGHT 0.75 // meters above floor -- TODO: this is a multiCam property, should not be defined here ... putting it on interface is too expensive since it never changes -- maybe we should retrieve the config value via some library?


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
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
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

void RTDBInputAdapter::getVisionFrame()
{
    (void)_rtdb->get(VISION_FRAME, &_visionFrame);
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
    int r = _rtdb->get(ROBOT_STATE, &robotState, robotId);
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


int RTDBInputAdapter::fillObjectCandidatesML(std::vector<objectMeasurement> &candidates, std::string const &objectType)
{
    // assume _visionFrame already retrieved, here we just convert data
    //candidates.clear(); // no, this function may be called multiple times per objectType 
    // TODO: make use of _visionFrame.frame counter, ignore dupes (sampling rates), etc.
    for (auto it = _visionFrame.objects.begin(); it != _visionFrame.objects.end(); ++it)
    {
        if (it->className == objectType) // obstacle, human, ball, ...
        {
            objectMeasurement candidate;
            candidate.identifier.robotID = _visionFrame.robot;
            static int counter = 0;
            candidate.identifier.uniqueID = counter++;
            candidate.timestamp = rtime::now(); // HACK - we should take _visionFrame.timestamp but it can be more than a second too old w.r.t. robot clock
            candidate.source = cameraEnum::MACHINELEARNING;
            candidate.confidence = it->confidence;
            candidate.azimuth = it->azimuth;
            candidate.elevation = it->elevation;
            candidate.radius = it->radius;
            candidate.cameraX = 0.0;
            candidate.cameraY = 0.0;
            candidate.cameraZ = CAMERA_HEIGHT;
            candidate.cameraPhi = 0.0;
            candidate.color = objectColorEnum::UNKNOWN; // TODO cleanup? solve via className and training?
            candidates.push_back(candidate);
        }
    }
    return RTDB2_SUCCESS;
}

int RTDBInputAdapter::fillBallCandidatesOld(T_BALL_CANDIDATES &candidates)
{
    T_BALL_CANDIDATES temp_candidates;
    int r = _rtdb->get(BALL_CANDIDATES, &temp_candidates);

    if (r == RTDB2_SUCCESS)
    {
        for (auto& c : temp_candidates)
        {
            candidates.push_back(c);
        }
    }

    return r;
}

int RTDBInputAdapter::getBallCandidates()
{
    // Acquire candidates from vision source(s)
    int r = !RTDB2_SUCCESS;
    
    _ballCandidates.clear();

    // The &= is effectively an "or" operation, because return 0 means success
    r &= fillObjectCandidatesML(_ballCandidates, "ball");
    r &= fillBallCandidatesOld(_ballCandidates);
    
    return r;
}

void RTDBInputAdapter::processBallCandidates()
{
    TRACE_FUNCTION("");
    // only continue if valid loc
    if (!_robotAdmin->inplay())
    {
        return;
    }

    // Acquire candidates from vision source(s)
    int r = getBallCandidates();
    
    // First put the Local FCS data to RTDB
    T_BALL_CANDIDATES_FCS measurementsFcs;
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

            // clear measurements here so that when this is being stimulated all data can be read
            // from rtdb instead of calculated
            measurementsFcs.clear();
        }
    }
 

    // Then get the Shared FCS data from RTDB
    std::vector<int>::const_iterator it;
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        // skipping self is only needed when sharevision if false, as then the data will not be on rtdb
        if (agentId != _myRobotId || _config.shareVision)
        {
            // Skip if configured to ignore
            if (!useVisionFromRobot(agentId))
            {
                continue;
            }

            T_BALL_CANDIDATES_FCS sharedMeasurements;
            r = _rtdb->get(BALL_CANDIDATES_FCS, &sharedMeasurements, agentId);

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

int RTDBInputAdapter::fillObstacleCandidatesOld(T_OBSTACLE_CANDIDATES &candidates)
{
    T_OBSTACLE_CANDIDATES temp_candidates;
    int r = _rtdb->get(OBSTACLE_CANDIDATES, &temp_candidates);

    if (r == RTDB2_SUCCESS)
    {
        for (auto& c : temp_candidates)
        {
            candidates.push_back(c);
        }
    }

    return r;
}

int RTDBInputAdapter::getObstacleCandidates()
{
    // Acquire candidates from vision source(s)
    int r = !RTDB2_SUCCESS;
    
    _obstacleCandidates.clear();

    // The &= is effectively an "or" operation, because return 0 means success
    r &= fillObjectCandidatesML(_obstacleCandidates, "obstacle");
    r &= fillObjectCandidatesML(_obstacleCandidates, "human");
    // TODO: how to make sure this list is complete? we now depend on ML internal tagging...
    r &= fillObstacleCandidatesOld(_obstacleCandidates);
    
    return r;
}

void RTDBInputAdapter::processObstacleCandidates()
{
    TRACE_FUNCTION("");
    // only continue if valid loc
    if (!_robotAdmin->inplay())
    {
        return;
    }
    
    // Acquire candidates from vision source(s)
    int r = getObstacleCandidates();

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
                
                // clear measurements here so that when this is being stimulated all data can be read
                // from rtdb instead of calculated
                measurementsFcs.clear();
            }
        }
    }

    // Then get the Shared FCS candidates from RTDB from other robots
    std::vector<int>::const_iterator it;
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        // skipping self is only needed when sharevision if false, as then the data will not be on rtdb
        if (agentId != _myRobotId || _config.shareVision)
        {
            // Skip if configured to ignore
            if (!useVisionFromRobot(agentId))
            {
                continue;
            }

            T_OBSTACLE_CANDIDATES_FCS sharedMeasurements;
            r = _rtdb->get(OBSTACLE_CANDIDATES_FCS, &sharedMeasurements, agentId);

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


int RTDBInputAdapter::fillLocalizationCandidatesML(T_LOCALIZATION_CANDIDATES &candidates, std::string const &objectType)
{
    // assume _visionFrame already retrieved, here we just convert data
    //candidates.clear(); // no, this function may be called multiple times per objectType 
    // TODO: make use of _visionFrame.frame counter, ignore dupes (sampling rates), etc.
    for (auto it = _visionFrame.objects.begin(); it != _visionFrame.objects.end(); ++it)
    {
        if (it->className == objectType)
        {
            robotLocalizationMeasurement localization_candidate;

            localization_candidate.identifier.robotID = _visionFrame.robot;
            localization_candidate.timestamp = rtime::now(); // HACK - we should take _visionFrame.timestamp but it can be more than a second too old w.r.t. robot clock
            localization_candidate.x = it->xCenter;
            localization_candidate.y = it->yCenter;
            localization_candidate.theta = it->azimuth;
            localization_candidate.confidence = it->confidence;

            candidates.push_back(localization_candidate);
        }
    }
    return RTDB2_SUCCESS;
}


int RTDBInputAdapter::fillLocalizationCandidatesOld(T_LOCALIZATION_CANDIDATES &candidates)
{
    T_LOCALIZATION_CANDIDATES temp_candidates;
    int r = _rtdb->get(LOCALIZATION_CANDIDATES, &temp_candidates);

    if (r == RTDB2_SUCCESS)
    {
        for (auto& c : temp_candidates)
        {
            candidates.push_back(c);
        }
    }

    return r;
}

int RTDBInputAdapter::getLocalizationCandidates()
{
    // Acquire candidates from vision source(s)
    int r = !RTDB2_SUCCESS;
    
    _localizationCandidates.clear();

    // The &= is effectively an "or" operation, because return 0 means success
    r &= fillLocalizationCandidatesML(_localizationCandidates, "location");
    r &= fillLocalizationCandidatesOld(_localizationCandidates);
    
    return r;
}

void RTDBInputAdapter::processLocalizationCandidates()
{
    TRACE_FUNCTION("");

    int r = getLocalizationCandidates();
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
