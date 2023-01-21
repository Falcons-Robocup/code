// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotAdministrator.cpp
 *
 *  Created on: Aug 18, 2016
 *      Author: Tim Kouters
 */

#include <boost/lexical_cast.hpp>

#include "int/administrators/robotAdministrator.hpp"

#include "cDiagnostics.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"

robotAdministrator::robotAdministrator(WorldModelConfig& wmConfig)
    : _wmConfig(wmConfig),
      _localizationAlgorithm(&wmConfig)
/*!
 * \brief Administrates robot measurements
 *
 * It has 2 purpose:
 *  - Separating local and remote measurements for syncing
 *  - Overruling own robot location for testing purposes
 *
 */
{
    _ownRobotID = getRobotNumber();
    _robots.clear();
    _isSimulated = isSimulatedEnvironment();
    _ballPossessionBallHandlers = false;
    _ballPossessionVision = false;
    _ballPossession = false;
    _prevBallPossession = false;
    _ballClaimedX = 0.0;
    _ballClaimedY = 0.0;
    _robotStatus.clear();
    _isLocationValid = false;
    _inplay = false;
    _wasInplay = false;
}

robotAdministrator::~robotAdministrator()
// Chuck Norris can hear sign language
{

}

void robotAdministrator::appendRobotVisionMeasurements(const std::vector<robotMeasurementClass_t> measurements)
{
    //TRACE("> count=%d", (int)measurements.size());
    try
    {
        for(auto itMeasurement = measurements.begin(); itMeasurement != measurements.end(); itMeasurement++)
        {
            // Verify correct measurement is received
            if(itMeasurement->getID().robotID != _ownRobotID)
            {
                TRACE_ERROR("Received vision measurement of other robot. OwnID: %d, receivedID: %d",
                        _ownRobotID, itMeasurement->getID().robotID);
            }
            else
            {
                // In the past, here we would add both the original vision candidate as well as the mirror, 
                // but now this symmetry is handled within localizationTracker
                _localizationAlgorithm.addVisionMeasurement(*itMeasurement);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::appendRobotDisplacementMeasurements(const std::vector<robotDisplacementClass_t> displacements)
{
    //TRACE(">");
    try
    {
        for(auto itDisplacement = displacements.begin(); itDisplacement != displacements.end(); itDisplacement++)
        {
            // Verify correct displacement is received
            if(itDisplacement->getID().robotID != _ownRobotID)
            {
                TRACE_ERROR("Received displacement of other robot. OwnID: %d, receivedID: %d",
                        _ownRobotID, itDisplacement->getID().robotID);
            }
            else
            {
                // Add displacement to localization algorithm
                _localizationAlgorithm.addDisplacement(*itDisplacement);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::appendRobotVelocityMeasurements(const std::vector<robotVelocityClass_t> velocities)
{
    //TRACE(">");
    try
    {
        for(auto itVelocity = velocities.begin(); itVelocity != velocities.end(); itVelocity++)
        {
            // Verify correct displacement is received
            if(itVelocity->getID().robotID != _ownRobotID)
            {
                TRACE_ERROR("Received velocity of other robot. OwnID: %d, receivedID: %d",
                        _ownRobotID, itVelocity->getID().robotID);
            }
            else
            {
                // Add displacement to localization algorithm
                _localizationAlgorithm.addVelocity(*itVelocity);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::updateRobotPositionAndVelocity(const robotClass_t robot)
/*!
 * \brief Update the position and velocity of a given robot
 * Be aware that this is a one on one copy without an algorithm in between
 * Reason is that the own robot has already calculate their own position.
 *
 * If used for own robot, the calculation will be overruled.
 * Thus can be used for testing purposes.
 */
{
    //TRACE("> id=%d", (int)robot.getRobotID());
    try
    {
        // Update member with new position and velocity
        _robots[robot.getRobotID()] = robot;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::disableOverrulingOfLocalRobot()
{
    //TRACE(">");
    if(_robots.find(_ownRobotID) != _robots.end())
    {
        _robots.erase(_ownRobotID);
    }

    if(_robotStatus.find(_ownRobotID) != _robotStatus.end())
    {
        _robotStatus.erase(_ownRobotID);
    }
    //TRACE("<");
}

void robotAdministrator::setRobotStatus(const uint8_t robotID, const robotStatusType status, rtime const timeNow)
{
    //TRACE(">");
    try
    {
        TRACE("setRobotStatus robotID=%d status=%d", robotID, (int)status);
        _robotStatus[robotID] = status;
        // when out of play, robot administrator does not communicate data
        // when GOING in play, we need ALSO a valid lock, so we poke _localizationAlgorithm

        if((status == robotStatusType::INPLAY) && (robotID == _ownRobotID) && !_isSimulated)
        {
            _localizationAlgorithm = robotLocalization(&_wmConfig);
        }

    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::performCalculation(rtime const timeNow)
/*!
 * \brief Perform the worldmodel calculations
 */
{
    std::ostringstream ss;
    ss << "timeNow: " << double(timeNow);
    TRACE_FUNCTION(ss.str().c_str());
    try
    {
        determineInPlay();
        if (!_isSimulated)
        {
            _localizationAlgorithm.calculatePositionAndVelocity(timeNow);
            _isLocationValid = _localizationAlgorithm.isValid();
            removeTimedoutRobots(timeNow);
            calcBallPossession(timeNow);
            // TODO redesign. time should not be passed around. 'local robot' and overruling is messy. Remove simulation bool. etc.
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

bool robotAdministrator::inplay()
{
    // overrule for coach: always inplay; we always want to see balls & obstacles in worldModel
    if (_ownRobotID == 0)
    {
        return true;
    }
    return _inplay;
}

robotClass_t robotAdministrator::getLocalRobotPosition(double timestamp)
/*!
 * \brief Fetch local robot position and velocity
 * This function can return 2 different robot positions depending whether
 * the own robot is overruled or not
 * 1) If overruled: Give back the overruled local position and velocity
 * 2) It NOT overruled (normal use-case) then give back the calculated local position and velocity
 */
{
    TRACE_FUNCTION("");
    try
    {
        robotClass_t ownRobot;

        if(_robots.find(_ownRobotID) != _robots.end())
        {
            // Own robot ID present in mapping of overruled robots
            ownRobot = _robots.at(_ownRobotID);
        }
        else
        {
            // Get new robot position
            ownRobot = _localizationAlgorithm.getRobotPositionAndVelocity(timestamp);
        }

        return ownRobot;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

std::vector<robotClass_t> robotAdministrator::getTeammembers()
{
    //TRACE(">");
    try
    {
        std::vector<robotClass_t> members;

        for(auto it = _robots.begin(); it !=_robots.end(); it++)
        {
            if(it->second.getRobotID() != _ownRobotID)
            {
                members.push_back(it->second);
            }
        }

        // Sort members on increasing robot ID
        std::sort(members.begin(), members.end(), robotClass_t::sortOnIncreasingRobotID);

        return members;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void robotAdministrator::determineInPlay()
{
    _inplay = (_robotStatus[_ownRobotID] == robotStatusType::INPLAY) && (_isLocationValid || _isSimulated);
    // log state change, to show in visualizer
    if (_inplay && !_wasInplay)
    {
        TRACE_INFO("software is INPLAY");
    }
    if (!_inplay && _wasInplay)
    {
        TRACE_INFO("software is OUTOFPLAY");
    }
    _wasInplay = _inplay;
}

std::vector<uint8_t> robotAdministrator::getActiveMembers()
{
    std::vector<uint8_t> members;
    try
    {

        for(auto it = _robotStatus.begin(); it != _robotStatus.end(); it++)
        {
            if(it->first == _ownRobotID)
            {
                if(it->second == robotStatusType::INPLAY)
                {
                    if (_isLocationValid || _isSimulated)
                    {
                        members.push_back(it->first);
                    }
                }
            }
            else if(it->second == robotStatusType::INPLAY)
            {
                members.push_back(it->first);
            }
        }

        // Sort members
        std::sort(members.begin(), members.end());

    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return members;
}

void robotAdministrator::removeTimedoutRobots(rtime const timeNow)
{
    TRACE_FUNCTION("");
    try
    {
        double maxTimeToLive = _wmConfig.getConfiguration().administration.teammember_timeout;

        for(auto i = _robots.begin(); i != _robots.end(); )
        {
            double time_diff = double(timeNow) - double(i->second.getTimestamp());
            if((time_diff > maxTimeToLive) && (i->first != _ownRobotID))
            {
                // Remove active robot as well
                if(_robotStatus.find(i->first) != _robotStatus.end())
                {
                    _robotStatus.erase(i->first);
                }

                // Delete robot with iterator
                i = _robots.erase(i);
            }
            else
            {
                i++;
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool robotAdministrator::getBallPossession() const
{
    return _ballPossession;
}

void robotAdministrator::calcBallPossession(double timestamp)
{
    _ballPossession = _ballPossessionVision && _ballPossessionBallHandlers;
    // check for state change, for storing claimed position (dribble rule)
    if (_ballPossession && !_prevBallPossession && _isLocationValid)
    {
        robotClass_t ownRobot = getLocalRobotPosition(timestamp); // TODO this is messy
        _ballClaimedX = ownRobot.getX();
        _ballClaimedY = ownRobot.getY();
    }
    if (_ballPossession != _prevBallPossession)
    {
        _prevBallPossession = _ballPossession;
    }
}

void robotAdministrator::getBallClaimedPosition(float &x, float &y)
{
    x = _ballClaimedX;
    y = _ballClaimedY;
}

void robotAdministrator::fillDiagnostics(diagWorldModel &diagnostics)
{
    TRACE_FUNCTION("");
    // activity
    diagnostics.shared.inplay = _inplay;
    diagnostics.shared.teamActivity = "";
    for (auto it = _robotStatus.begin(); it != _robotStatus.end(); it++)
    {
        if (it->second == robotStatusType::INPLAY)
        {
            diagnostics.shared.teamActivity += boost::lexical_cast<std::string>((int)(it->first)) + " ";
        }
    }
    // localization
    localizationDiagnostics_t locDiagStruct = _localizationAlgorithm.getDiagnostics();
    diagnostics.shared.numVisionCandidates = locDiagStruct.numVisionCandidates;
    diagnostics.shared.visionLocAge = locDiagStruct.visionLocAge;
    // generate warning in case vision is running dry (e.g. when positioning for throwin, or robot is tilted, or something wrong with assy)
    if (diagnostics.shared.inplay && diagnostics.shared.visionLocAge > 6.0) // TODO make reconfigurable
    {
        TRACE_WARNING_TIMEOUT(6.0, "vision lock lost -- driving blind");
    }
    diagnostics.shared.numMotorDisplacementSamples = locDiagStruct.numMotorDisplacementSamples;
    diagnostics.shared.bestVisionCandidate = locDiagStruct.bestVisionCandidate;
    diagnostics.shared.visionConfidence = locDiagStruct.confidence;
    diagnostics.shared.visionNoiseXY = locDiagStruct.visionNoiseXY;
    diagnostics.shared.visionNoisePhi = locDiagStruct.visionNoisePhi;
    diagnostics.shared.locationValid = locDiagStruct.isValid;
    // ball possession
    diagnostics.shared.ballPossessionBallHandlers = _ballPossessionBallHandlers;
    diagnostics.shared.ballPossessionVision = _ballPossessionVision;
}

