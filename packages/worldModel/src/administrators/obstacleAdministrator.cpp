 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleAdministrator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */


#include "int/administrators/obstacleAdministrator.hpp"

#include "int/configurators/administrationConfigurator.hpp"
#include "int/configurators/obstacleTrackerConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"
#include "timeConvert.hpp"

obstacleAdministrator::obstacleAdministrator()
/*!
 * \brief Administrates obstacle measurements
 *
 * It has 2 purposes:
 *  - Separating local and remote measurements for syncing
 *  - Making sure measurements are only fed once to the discriminator
 *
 */
{
	_ownRobotID = getRobotNumber();
	_obstacleMeasurements.clear();
    _diagSender = NULL;
    enableDiagnostics();
}

obstacleAdministrator::~obstacleAdministrator()
/*
 * When Chuck Norris was born he drove his mom home from the hospital.
 */
{

}

void obstacleAdministrator::appendObstacleMeasurements(const std::vector<obstacleMeasurementType> measurements)
{
    //TRACE("> #meas=%d", (int)measurements.size());
	try
	{
		for(auto itCandidate = measurements.begin(); itCandidate != measurements.end(); itCandidate++)
		{
			auto it = _obstacleMeasurements.find(itCandidate->getID());

			/*
			 * If not found, add it and feed to object discriminator
			 */
			if(it == _obstacleMeasurements.end())
			{
				_obstacleMeasurements[itCandidate->getID()] = (*itCandidate);
				_obstacleDiscriminator.addMeasurement(*itCandidate);
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

void obstacleAdministrator::overruleObstacles(const std::vector<obstacleClass_t> obstacles)
{
    TRACE("> #overruled=%d  #obstacles=%d", (int)_overruledObstacles.size(), (int)obstacles.size());
	try
	{
		if(!obstacles.empty())
		{
			_overruledObstacles[obstacles.at(0).getId()] = obstacles;
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    TRACE("< #overruled=%d", (int)_overruledObstacles.size());
}

void obstacleAdministrator::getLocalObstacleMeasurements(std::vector<obstacleMeasurementType> &measurements)
{
    //TRACE(">");
	try
	{
		measurements.clear();

		for(auto it = _obstacleMeasurements.begin(); it != _obstacleMeasurements.end(); it++)
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

void obstacleAdministrator::notifyOwnLocation(robotClass_t const &ownLocation)
{
    // store so it can be used in filterOutTeamMembers
    _ownPos.x = ownLocation.getX();
    _ownPos.y = ownLocation.getY();
}

void obstacleAdministrator::notifyTeamMembers(std::vector<robotClass_t> const &teamMembers)
{
    // store so it can be used in filterOutTeamMembers
    _teamMembers.clear();
    for (auto it = teamMembers.begin(); it != teamMembers.end(); ++it)
    {
        _teamMembers.push_back(Point2D(it->getX(), it->getY()));
    }
}

void obstacleAdministrator::filterOutTeamMembers(std::vector<obstacleClass_t> &obstacles)
{   
    // this filter function was moved and adapted from worldModelInfoUpdater, to solve #482 a.o.

    try
    {
        // NOTE: we do not anymore reuse (abuse) trackerXY tolerance
        // because tracking is a fundamentally different thing than filtering
        // filtering is merely post-processing tracker results
        float filterXYownTolerance = obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::
filterXYownTolerance);
        float filterXYmemberTolerance = obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::filterXYmemberTolerance);

        for (auto itObst = obstacles.begin(); itObst != obstacles.end(); )
        {
            bool found = false;
            Point2D obstaclePos = Point2D(itObst->getX(), itObst->getY());
            
            // check against own position
            if (vectorsize(_ownPos - obstaclePos) < filterXYownTolerance)
            {
                found = true;
            }

            // check against team member positions
            for (auto itMember = _teamMembers.begin(); ((itMember != _teamMembers.end()) && (!found)); itMember++)
            {
                if (vectorsize(*itMember - obstaclePos) < filterXYmemberTolerance)
                {
                    found = true;
                }
            }

            if (found)
            {
                itObst = obstacles.erase(itObst);
            }
            else
            {
                itObst++;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void obstacleAdministrator::performCalculation(const double timeNow)
{
    TRACE("> t=%16.6f  #overrule=%d", timeNow, (int)_overruledObstacles.size());
	try
	{
		cleanUpTimedOutObstacleMeasurements(timeNow);

        // in case of simulation, _overruledObstacles is used instead of _obstacleDiscriminator
	    if (_overruledObstacles.size() == 0)
	    {
	        // real mode, not simulation - calculate
		    _obstacleDiscriminator.performCalculation(timeNow);
		    // get result, filter teammembers and own position, cache for getObstacles()
		    _resultObstacles = _obstacleDiscriminator.getObstacles();
		    filterOutTeamMembers(_resultObstacles);
		    // diagnostics
		    sendDiagnostics(_resultObstacles);
	    }
	    else
	    {
	        // send the simulated obstacles(s) to visualizer
	        std::vector<obstacleClass_t> obstacles;
			for(auto it = _overruledObstacles.begin(); it != _overruledObstacles.end(); it++)
			{
    			for(auto it2 = it->second.begin(); it2 != it->second.end(); it2++)
			    {
			        TRACE("obst=[%6.2f %6.2f]", it2->getX(), it2->getY());
    				obstacles.push_back(*it2);
				}
			}
			sendDiagnostics(obstacles);
	    }
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
    TRACE("<");
}

void obstacleAdministrator::getObstacles(std::vector<obstacleClass_t> &obstacles)
{
    //TRACE(">");
	try
	{
		// in case of simulation, _overruledObstacles is used instead of _obstacleDiscriminator
		if(_overruledObstacles.empty())
		{
			obstacles = _resultObstacles;
		}
		else
		{
			for(auto it = _overruledObstacles.begin(); it != _overruledObstacles.end(); it++)
			{
				std::vector<obstacleClass_t> obstacleVector = it->second;

				for(auto itObstacle = obstacleVector.begin(); itObstacle != obstacleVector.end(); itObstacle++)
				{
					obstacles.push_back(*itObstacle);
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
    //TRACE("< numobstacles=%d", (int)obstacles.size());
}

void obstacleAdministrator::cleanUpTimedOutObstacleMeasurements(const double timeNow)
{
    //TRACE("> size=%d", (int)_obstacleMeasurements.size());
	try
	{
		double maxTimeToLive = administrationConfigurator::getInstance().getObstacleTimeToLive();

		for(auto i = _obstacleMeasurements.begin(); i != _obstacleMeasurements.end();)
		{
			double time_diff = timeNow - i->second.getTimestamp();
			if(time_diff > maxTimeToLive)
            {
                // Delete obstacle candidate
				i = _obstacleMeasurements.erase(i);
            }
			else
			{
				i++;
			}
		}

		for(auto it = _overruledObstacles.begin(); it != _overruledObstacles.end();)
		{

			if(it->second.empty())
			{
				it = _overruledObstacles.erase(it);
			}
			else
			{
				double time_diff = timeNow - it->second.at(0).getTimestamp();
				if(time_diff > maxTimeToLive)
				{
					// Delete obstacle candidate
					it = _overruledObstacles.erase(it);
				}
				else
				{
					it++;
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
    //int removed = origSize - (int)_obstacleMeasurements.size();
    //TRACE("< removed=%d", removed);
}

void obstacleAdministrator::enableDiagnostics()
{
    _diagSender = new diagnostics::cDiagnosticsSender<rosMsgs::t_diag_wm_obstacles>(diagnostics::DIAG_WM_OBST, 10, false);
}

void obstacleAdministrator::sendDiagnostics(std::vector<obstacleClass_t> const &obstacles)
{
    if (_diagSender)
    {
		rosMsgs::t_diag_wm_obstacles diagMsg;
		diagMsg.numTrackers = (int)obstacles.size();
		for(auto it = obstacles.begin(); it != obstacles.end(); it++)
		{
            rosMsgs::t_obstacle obstMsg;
            obstacleClass_t obstacle = *it;
            obstMsg.id = obstacle.getId();
            obstMsg.x = obstacle.getX();
            obstMsg.y = obstacle.getY();
            obstMsg.vx = obstacle.getVX();
            obstMsg.vy = obstacle.getVY();
            obstMsg.confidence = obstacle.getConfidence();
            diagMsg.obstacles.push_back(obstMsg);
        }
        _diagSender->set(diagMsg);
    }
}

