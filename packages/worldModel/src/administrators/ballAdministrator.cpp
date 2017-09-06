 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballAdministrator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballAdministrator.hpp"

#include <algorithm>

#include "int/configurators/administrationConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"
#include "timeConvert.hpp"

ballAdministrator::ballAdministrator()
/*!
 * \brief Administrates ball measurements
 *
 * It has 2 purposes:
 *  - Separating local and remote measurements for syncing
 *  - Making sure measurements are only fed once to the discriminator
 *
 */
{
	_ownRobotID = getRobotNumber();
	_ballMeasurements.clear();
	_overruledBalls.clear();
	
	_globalBallDiscriminator.enableDiagnostics();
}

ballAdministrator::~ballAdministrator()
/*
 * Chuck Norris' calendar goes straight from March 31st to April 2nd.
 * No one fools Chuck Norris.
 */
{

}

void ballAdministrator::appendBallMeasurements(const std::vector<ballMeasurementType> measurements)
{
    //TRACE("> #meas=%d", (int)measurements.size());
	try
	{
		for(auto itMeasurement = measurements.begin(); itMeasurement != measurements.end(); itMeasurement++)
		{
			auto it = _ballMeasurements.find(itMeasurement->getID());

			/*
			 * If not found, add it and feed to object discriminator
			 */
			if(it == _ballMeasurements.end())
			{
				/* Only add own measurements to local discriminator */
				if(itMeasurement->getID().robotID == _ownRobotID)
				{
					_localBallDiscriminator.addMeasurement(*itMeasurement);
				}

				_ballMeasurements[itMeasurement->getID()] = (*itMeasurement);
				_globalBallDiscriminator.addMeasurement(*itMeasurement);
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

void ballAdministrator::overruleBall(const ballClass_t ball)
{
	try
	{
		_overruledBalls[ball.getId()] = ball;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballAdministrator::getLocalBallMeasurements(std::vector<ballMeasurementType> &measurements)
{
    //TRACE(">");
	try
	{
		measurements.clear();

		for(auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); it++)
		{
			if(it->first.robotID == _ownRobotID)
			{
				measurements.push_back(it->second);
			}
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("< #meas=%d", (int)measurements.size());
}

void ballAdministrator::performCalculation(const double timeNow)
{
    //TRACE("> t=%16.6f", timeNow);
	try
	{
        // in case of simulation, _overruledBalls is used instead of _ballDiscriminator
	    if (_overruledBalls.size() == 0)
	    {
		    cleanUpTimedOutBallMeasurements(timeNow);
		    _globalBallDiscriminator.performCalculation(timeNow);
		    _localBallDiscriminator.performCalculation(timeNow);
	    }
	    else
	    {
	        // send the simulated ball(s) to visualizer
	        std::vector<ballClass_t> balls;
			for(auto it = _overruledBalls.begin(); it != _overruledBalls.end(); it++)
			{
				balls.push_back(it->second);
			}
	        _globalBallDiscriminator.sendSimDiagnostics(balls);
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

void ballAdministrator::getOwnBalls(std::vector<ballClass_t> &balls)
{
    //TRACE(">");
	try
	{
		if(_overruledBalls.empty())
		{
			balls = _localBallDiscriminator.getBalls();
		}
		else
		{
			for(auto it = _overruledBalls.begin(); it != _overruledBalls.end(); it++)
			{
				balls.push_back(it->second);
			}
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("< numballs=%d", (int)balls.size());
}

void ballAdministrator::getGlobalBalls(std::vector<ballClass_t> &balls)
{
    //TRACE(">");
	try
	{
		if(_overruledBalls.empty())
		{
			balls = _globalBallDiscriminator.getBalls();
		}
		else
		{
			for(auto it = _overruledBalls.begin(); it != _overruledBalls.end(); it++)
			{
				balls.push_back(it->second);
			}
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    //TRACE("< numballs=%d", (int)balls.size());
}

void ballAdministrator::cleanUpTimedOutBallMeasurements(const double timeNow)
{
    //TRACE("> size=%d", (int)_ballMeasurements.size());
	try
	{
		double maxTimeToLive = administrationConfigurator::getInstance().getBallTimeToLive();

		for(auto i = _ballMeasurements.begin(); i != _ballMeasurements.end(); )
		{
			double time_diff = timeNow - i->second.getTimestamp();
			if(time_diff > maxTimeToLive)
            {
                // Delete ball measurement
				i = _ballMeasurements.erase(i);
            }
			else
			{
				i++;
			}
		}

		for(auto i = _overruledBalls.begin(); i != _overruledBalls.end(); )
		{
			double time_diff = timeNow - i->second.getTimestamp();
			if(time_diff > maxTimeToLive)
            {
                // Delete robot with iterator
				i = _overruledBalls.erase(i);
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
    //int removed = (int)_ballMeasurements.size() - origSize;
    //TRACE("< removed=%d", removed);
}
