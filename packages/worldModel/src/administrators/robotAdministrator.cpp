 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotAdministrator.cpp
 *
 *  Created on: Aug 18, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/robotAdministrator.hpp"

#include "int/configurators/administrationConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"
#include "timeConvert.hpp"

robotAdministrator::robotAdministrator()
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
	_ballPossessions.clear();
	_robotStatus.clear();
	_isLocationValid = false;
    _diagSender = NULL;
    enableDiagnostics(); // TODO if this class is reused, then explicitly only enable diagnostics for own robot
}

robotAdministrator::~robotAdministrator()
/*
 * Chuck Norris can hear sign language
 */
{

}

void robotAdministrator::enableDiagnostics()
{
    _diagSender = new diagnostics::cDiagnosticsSender<rosMsgs::t_diag_wm_loc>(diagnostics::DIAG_WM_LOC, 10, false);
}

void robotAdministrator::appendRobotVisionMeasurements(const std::vector<robotMeasurementClass_t> measurements)
{
    //TRACE("> count=%d", (int)measurements.size());
	try
	{
		for(auto itMeasurement = measurements.begin(); itMeasurement != measurements.end(); itMeasurement++)
		{
			/*
			 * Verify correct measurement is received
			 */
			if(itMeasurement->getID().robotID != _ownRobotID)
			{
				TRACE_ERROR("Received vision measurement of other robot. OwnID: %d, receivedID: %d",
						_ownRobotID, itMeasurement->getID().robotID);
			}
			else
			{
				/*
				 * In the past, here we would add both the original vision candidate as well as the mirror, 
				 * but now this symmetry is handled within localizationTracker
 				 */
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
			/*
			 * Verify correct displacement is received
			 */
			if(itDisplacement->getID().robotID != _ownRobotID)
			{
				TRACE_ERROR("Received displacement of other robot. OwnID: %d, receivedID: %d",
						_ownRobotID, itDisplacement->getID().robotID);
			}
			else
			{
				/*
				 * Add displacement to localization algorithm
				 */
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
		/*
		 * Update member with new position and velocity
		 */
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

	if(_ballPossessions.find(_ownRobotID) != _ballPossessions.end())
	{
		_ballPossessions.erase(_ownRobotID);
	}

	if(_robotStatus.find(_ownRobotID) != _robotStatus.end())
	{
		_robotStatus.erase(_ownRobotID);
	}
    //TRACE("<");
}

void robotAdministrator::claimBallPossession(const uint8_t robotID, const ballClaimType claimType)
/*!
 * \brief Claim the ball from multiple sources
 * 1) Verify robotID is present in mapping
 *  a) If not present add to mapping
 *  b) if remote ID, then already set camera claim to true
 * 2) See what type of claim it is
 *  a) If camera claim, do a camera claim call (will only happen with own robotID)
 *  b) If ball handlers claim, do a ball handlers claim with the current position
 */
{
    //TRACE(">");
	try
	{
		if(_ballPossessions.find(robotID) == _ballPossessions.end())
		{
			ballPossessionClass_t ballPossession = ballPossessionClass_t(robotID);

			// 1a
			if(robotID != _ownRobotID)
			{
				ballPossession.claimBallByCamera();
			}

			// 1b
			_ballPossessions[robotID] = ballPossession;
		}

		// 2
		switch(claimType)
		{
			// 2a
			case ballClaimType::CAMERA:
			{
				_ballPossessions.at(robotID).claimBallByCamera();
				break;
			}

			// 2b
			case ballClaimType::BALL_HANDLERS:
			{
				if(robotID == _ownRobotID)
				{
					robotClass_t robot = getLocalRobotPosition();
					_ballPossessions.at(robotID).claimBallByBallHandlers(robot.getX(), robot.getY());
				}
				else
				{
					if(_robots.find(robotID) != _robots.end())
					{
						robotClass_t robot = _robots.at(robotID);
						_ballPossessions.at(robotID).claimBallByBallHandlers(robot.getX(), robot.getY());
					}
					else
					{
						TRACE_ERROR("Ignoring ball claim of robot %d due to no position found",
								robotID);
					}
				}
				break;
			}

			default:
			{
				TRACE_ERROR("Ignoring ball claim of robot %d due to INVALID claim type",
						robotID);
				break;
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

void robotAdministrator::releaseBallPossession(const uint8_t robotID, const ballClaimType releaseType)
/*!
 * \brief Release the ball from multiple sources
 * 1) Verify robotID is present in mapping
 *  a) If not present skip function
 * 2) See what type of release it is
 *  a) If camera release, do a camera release call (will only happen with own robotID)
 *  b) If ball handlers release, do a ball handlers release
 */
{
    //TRACE("> robotID=%d", (int)robotID);
	try
	{
		// 1
		if(_ballPossessions.find(robotID) != _ballPossessions.end())
		{
			// 2
			switch(releaseType)
			{
				// 2a
				case ballClaimType::CAMERA:
				{
					_ballPossessions.at(robotID).releaseBallByCamera();
					break;
				}

				// 2b
				case ballClaimType::BALL_HANDLERS:
				{

					_ballPossessions.at(robotID).releaseBallByBallHandlers();
					break;
				}

				default:
				{
					TRACE_ERROR("Ignoring ball release of robot %d due to INVALID claim type",
							robotID);
					break;
				}
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

void robotAdministrator::setRobotStatus(const uint8_t robotID, const robotStatusType status, const double timeNow)
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
			_localizationAlgorithm = robotLocalization();
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

void robotAdministrator::performCalculation(const double timeNow)
/*!
 * \brief Perform the worldmodel calculations
 */
{
    //TRACE("> t=%16.6f", timeNow);
	try
	{
	    if (!_isSimulated)
	    {
		    _localizationAlgorithm.calculatePositionAndVelocity(timeNow);
            _isLocationValid = _localizationAlgorithm.isValid();
            sendDiagnostics();
		    removeTimedoutRobots(timeNow);
	    }
	    else
	    {
            sendDiagnostics();
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

robotClass_t robotAdministrator::getLocalRobotPosition()
/*!
 * \brief Fetch local robot position and velocity
 * This function can return 2 different robot positions depending whether
 * the own robot is overruled or not
 * 1) If overruled: Give back the overruled local position and velocity
 * 2) It NOT overruled (normal use-case) then give back the calculated local position and velocity
 */
{
    //TRACE(">");
	try
	{
		robotClass_t ownRobot;

		if(_robots.find(_ownRobotID) != _robots.end())
		{
			/*
			 * Own robot ID present in mapping of overruled robots
			 */
			ownRobot = _robots.at(_ownRobotID);
		}
		else
		{
			/*
			 * Get new robot position
			 */
			ownRobot = _localizationAlgorithm.getRobotPositionAndVelocity();
		}

		return ownRobot;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("<");
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

		/*
		 * Sort members on increasing robot ID
		 */
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

std::vector<uint8_t> robotAdministrator::getActiveMembers()
{
    //TRACE("> _robotStatus.size()=%d", (int)_robotStatus.size());
    std::string activeString;
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
	    				activeString += boost::lexical_cast<std::string>((int)it->first) + " ";
    				}
				}
			}
			else if(it->second == robotStatusType::INPLAY)
			{
				members.push_back(it->first);
				activeString += boost::lexical_cast<std::string>((int)it->first) + " ";
			}
		}

		/*
		 * Sort members
		 */
		std::sort(members.begin(), members.end());

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("< members.size()=%d active=[%s]", (int)members.size(), activeString.c_str());
	return members;
}

ballPossessionClass_t robotAdministrator::getLocalBallPossession()
{
    //TRACE(">");
	try
	{
		ballPossessionClass_t possession;

		if(_ballPossessions.find(_ownRobotID) != _ballPossessions.end())
		{
			possession = _ballPossessions.at(_ownRobotID);
		}

		return possession;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("<");
}

ballPossessionClass_t robotAdministrator::getBallPossession()
/*!
 * \brief Get ball possession
 * First look at own ball possession. If own robot has possession give prio
 * over other robots. This is done in code by the found boolean.
 */
{
    //TRACE(">");
	try
	{
		ballPossessionClass_t possession = getLocalBallPossession();
		bool found = possession.hasBallPossession();

		for(auto it = _ballPossessions.begin(); ((it != _ballPossessions.end()) && !found); it++)
		{
			if(it->second.hasBallPossession())
			{
				possession = it->second;
				found = true;
			}
		}

		return possession;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("<");
}

void robotAdministrator::removeTimedoutRobots(const double timeNow)
{
    //TRACE(">");
	try
	{
		double maxTimeToLive = administrationConfigurator::getInstance().getRobotTimeToLive();

		for(auto i = _robots.begin(); i != _robots.end(); )
		{
			double time_diff = timeNow - i->second.getTimestamp();
			if((time_diff > maxTimeToLive) && (i->first != _ownRobotID))
            {
				// Remove ball possession as well using key instead of iterator
				if(_ballPossessions.find(i->first) != _ballPossessions.end())
				{
					_ballPossessions.erase(i->first);
				}

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
    //TRACE("<");
}

void robotAdministrator::sendDiagnostics()
{
    if (_diagSender != NULL)
    {
        // convert internal struct to ROS message
        rosMsgs::t_diag_wm_loc msg;
        if (_isSimulated)
        {
            msg.isLocationValid = true; // by definition valid loc
            // get position from overruled administration
            robotClass_t pos = getLocalRobotPosition();
            msg.ownpos.x = pos.getX();
            msg.ownpos.y = pos.getY();
            msg.ownpos.phi = pos.getTheta();
            msg.ownpos.vx = pos.getVX();
            msg.ownpos.vy = pos.getVY();
            msg.ownpos.vphi = pos.getVTheta();
            // remainder: leave to ROS default values (typically zero)
        }
        else
        {
            // get data from algorithm
            localizationDiagnostics_t diagData = _localizationAlgorithm.getDiagnostics();
            // TODO consider making a facility for this -- currently error-prone to make&extend
            msg.isLocationValid = diagData.isValid;
            msg.numVisionCandidates = diagData.numVisionCandidates;
            msg.numMotorDisplacementSamples = diagData.numMotorDisplacementSamples;
            msg.confidence = diagData.confidence;
            msg.visionNoiseXY = diagData.visionNoiseXY;
            msg.visionNoisePhi = diagData.visionNoisePhi;
            msg.ownpos.x = diagData.ownpos.x;
            msg.ownpos.y = diagData.ownpos.y;
            msg.ownpos.phi = diagData.ownpos.phi;
            msg.ownpos.vx = diagData.ownpos.vx;
            msg.ownpos.vy = diagData.ownpos.vy;
            msg.ownpos.vphi = diagData.ownpos.vphi;
            msg.bestVisionCandidate.x = diagData.bestVisionCandidate.x;
            msg.bestVisionCandidate.y = diagData.bestVisionCandidate.y;
            msg.bestVisionCandidate.phi = diagData.bestVisionCandidate.phi;
            msg.bestVisionCandidate.confidence = diagData.bestVisionCandidate.confidence;
        }
        // update data, for low-frequent sender to pick up
        _diagSender->set(msg);
    }
}

