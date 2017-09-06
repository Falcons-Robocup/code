 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include "cDiagnosticsEvents.hpp"
#include "timeConvert.hpp"

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
	_wmInfo = NULL;
	_wmSyncInfo = NULL;
	_ballIsCaughtByBallHandlers = false;
	_robotStatus = robotStatusType::INVALID;
}

adaptersCollector::~adaptersCollector()
/*
 * The reason the Holy Grail has never been recovered is because nobody is
 * brave enough to ask Chuck Norris to give up his favourite coffee mug.
 */
{

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

void adaptersCollector::setWorldModelInfoUpdater(worldModelInfoUpdater *wmInfo)
{
	_wmInfo = wmInfo;
}
void adaptersCollector::setWorldModelSyncInfoUpdater(worldModelSyncInfoUpdater *wmSyncInfo)
{
	_wmSyncInfo = wmSyncInfo;
}

void adaptersCollector::updateVisionRobotMeasurement(const std::vector<robotMeasurementClass_t> measurement)
{
	try
	{
		if(_robotAdmin != NULL)
		{
			_robotAdmin->appendRobotVisionMeasurements(measurement);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

}

void adaptersCollector::updateVisionBallMeasurement(const std::vector<ballMeasurementType> measurement)
{
	try
	{
        if ((_ballAdmin != NULL) && (_robotAdmin != NULL))
        {
            // convert camera position from RCS to FCS for each ball measurement
            std::vector<ballMeasurementType> measurementFcs = measurement;
            robotClass_t pos = _robotAdmin->getLocalRobotPosition();
            Position2D robotPos(pos.getX(), pos.getY(), pos.getTheta());
            for (auto it = measurementFcs.begin(); it != measurementFcs.end(); ++it)
            {
                float camZ = it->getCameraZ();
                Position2D camRcs(it->getCameraX(), it->getCameraY(), it->getCameraPhi());
                Position2D camFcs = camRcs.transform_rcs2fcs(robotPos);
                it->setCameraOffset(camFcs.x, camFcs.y, camZ, camFcs.phi);
            }
            _ballAdmin->appendBallMeasurements(measurementFcs);
        }
        else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateVisionObstacleMeasurement(const std::vector<obstacleMeasurementType> measurement)
{
	try
	{
		if ((_obstacleAdmin != NULL) && (_robotAdmin != NULL))
		{
            // convert camera position from RCS to FCS for each obstacle measurement
            std::vector<obstacleMeasurementType> measurementFcs = measurement;
            robotClass_t pos = _robotAdmin->getLocalRobotPosition();
            Position2D robotPos(pos.getX(), pos.getY(), pos.getTheta());
            for (auto it = measurementFcs.begin(); it != measurementFcs.end(); ++it)
            {
                float camZ = it->getCameraZ();
                Position2D camRcs(it->getCameraX(), it->getCameraY(), it->getCameraPhi());
                Position2D camFcs = camRcs.transform_rcs2fcs(robotPos);
                it->setCameraOffset(camFcs.x, camFcs.y, camZ, camFcs.phi);
            }
			_obstacleAdmin->appendObstacleMeasurements(measurementFcs);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateVisionBallPossession(const bool ballIsSeenInFrontOfBallHandlers)
/*!
 * \brief Optimized function for claiming camera ball possession
 * Only call function when seen/not seen is switched
 */
{
	try
	{
		if(_robotAdmin != NULL)
		{
			if(ballIsSeenInFrontOfBallHandlers)
			{
				_robotAdmin->claimBallPossession(getRobotNumber(), ballClaimType::CAMERA);
			}
			else
			{
				_robotAdmin->releaseBallPossession(getRobotNumber(), ballClaimType::CAMERA);
			}
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updatePeripheralsDisplacement(const std::vector<robotDisplacementClass_t> displacements)
{
	try
	{
		if(_robotAdmin != NULL)
		{
			_robotAdmin->appendRobotDisplacementMeasurements(displacements);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updatePeripheralsBallPossession(const bool ballIsCaughtByBallHandlers)
/*!
 * \brief Optimized function for claiming ball possession
 * Only call function when caught / uncaught is switched
 */
{
	try
	{
		if((_robotAdmin != NULL) && (_ballIsCaughtByBallHandlers != ballIsCaughtByBallHandlers))
		{
			if(ballIsCaughtByBallHandlers)
			{
				_robotAdmin->claimBallPossession(getRobotNumber(), ballClaimType::BALL_HANDLERS);
			}
			else
			{
				_robotAdmin->releaseBallPossession(getRobotNumber(), ballClaimType::BALL_HANDLERS);
			}

			_ballIsCaughtByBallHandlers = ballIsCaughtByBallHandlers;
		}

		if(_robotAdmin == NULL)
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updatePeripheralsRobotStatus(const robotStatusType status)
/*!
 * \brief Optimized function for setting status
 * Only call function when in/out of play is switched
 */
{
	try
	{
		if((_robotAdmin != NULL) && (_robotStatus != status))
		{
			_robotAdmin->setRobotStatus(getRobotNumber(), status, getTimeNow());
			// generate an event, for eventlogging on coach
			if (status == robotStatusType::INPLAY)
			{
				TRACE_INFO("switched to INPLAY");
			}
			else
			{
				TRACE_INFO("switched to OUTOFPLAY");
			}
			_robotStatus = status;
		}

		if(_robotAdmin == NULL)
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateWmSyncBallMeasurement(const std::vector<ballMeasurementType> measurements)
{
	try
	{
		if(_ballAdmin != NULL)
		{
			_ballAdmin->appendBallMeasurements(measurements);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateWmSyncObstacleMeasurement(const std::vector<obstacleMeasurementType> measurements)
{
	try
	{
		if(_obstacleAdmin !=  NULL)
		{
			_obstacleAdmin->appendObstacleMeasurements(measurements);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateWmSyncBallPossession(const std::map<uint8_t, bool> possession)
{
	try
	{
		if(_robotAdmin != NULL)
		{
			for(auto it = possession.begin(); it != possession.end(); it++)
			{
				if(it->second)
				{
					_robotAdmin->claimBallPossession(it->first, ballClaimType::BALL_HANDLERS);
					_robotAdmin->claimBallPossession(it->first, ballClaimType::CAMERA);
				}
				else
				{
					_robotAdmin->releaseBallPossession(it->first, ballClaimType::BALL_HANDLERS);
					_robotAdmin->releaseBallPossession(it->first, ballClaimType::CAMERA);
				}
			}
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::updateWmSyncTeamMember(const robotClass_t robot)
{
	try
	{
		if(_robotAdmin != NULL)
		{
            double t = getTimeNow();
            robotClass_t robotWithTime = robot;
            robotWithTime.setTimestamp(t);
            _robotAdmin->updateRobotPositionAndVelocity(robotWithTime);
            /*
             * Set robot in-play since otherwise its not synced if not active
             */
            _robotAdmin->setRobotStatus(robot.getRobotID(), robotStatusType::INPLAY, t);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::heartBeatRecalculation(bool dummy)
{
	try
	{
		double timeNow = getTimeNow();
    	TRACE("> t=%16.6f", timeNow);

		if((_robotAdmin != NULL) &&
		   (_ballAdmin != NULL) &&
		   (_obstacleAdmin != NULL) &&
		   (_wmInfo != NULL) &&
		   (_wmSyncInfo != NULL))
		{
			/*
			 * Perform the calculations given current information
			 */
			_robotAdmin->performCalculation(timeNow);
			_ballAdmin->performCalculation(timeNow);
			_obstacleAdmin->performCalculation(timeNow);

			_wmInfo->notify();
			_wmSyncInfo->notify();
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

/*
 * Overruling functions
 */
void adaptersCollector::overruleRobotPosition(const robotClass_t robot)
{
	try
	{
		if(_robotAdmin != NULL)
		{
			_robotAdmin->updateRobotPositionAndVelocity(robot);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::overruleBallPosition(const ballClass_t ball)
{
	try
	{
		if(_ballAdmin != NULL)
		{
			_ballAdmin->overruleBall(ball);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::overruleObstaclePostions(const std::vector<obstacleClass_t> obstacles)
{
	try
	{
		if(_obstacleAdmin != NULL)
		{
			_obstacleAdmin->overruleObstacles(obstacles);
		}
		else
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void adaptersCollector::overruleRobotStatus(const robotStatusType status)
/*!
 * \brief Software overrule for inplay/outofplay switch
 * Useful for quick remote recovery from vision glitches.
 *
 * TODO: due to constant stimulation from the button, the status will not be persistent
 * for now this is fine, but for future use cases we may need to do something here
 */
{
	try
	{
		if((_robotAdmin != NULL) && (_robotStatus != status))
		{
			_robotAdmin->setRobotStatus(getRobotNumber(), status, getTimeNow());
			// generate an event, for eventlogging on coach
			if (status == robotStatusType::INPLAY)
			{
				TRACE_WARNING("overruled to INPLAY");
			}
			else
			{
				TRACE_WARNING("overruled to OUTOFPLAY");
			}
			_robotStatus = status;
		}

		if(_robotAdmin == NULL)
		{
			TRACE_ERROR("NULL pointer exception");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}


