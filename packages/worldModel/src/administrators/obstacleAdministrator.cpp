 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include "int/administrators/obstacleDiscriminator.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

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
    _obstacleDiscriminator = new obstacleDiscriminator();
}

obstacleAdministrator::~obstacleAdministrator()
// When Chuck Norris was born he drove his mom home from the hospital.
{
    delete _obstacleDiscriminator;
}

void obstacleAdministrator::appendObstacleMeasurements(const std::vector<obstacleMeasurement> measurements)
{
    TRACE("> #meas=%d", (int)measurements.size());
    try
    {
        // hack: keeper should never respond to nor sync out obstacles, especially now it has a fancy new keeper frame
        // we saw in testmatch 20180221 that several fake obstacles were projected in the penalty area
        // teammembers should not be scared of them
        // proper solution would include one or more configuration parameters, then make R1 specific yaml
        if (_ownRobotID == 1)
        {
            return;
        }

        for(auto itCandidate = measurements.begin(); itCandidate != measurements.end(); itCandidate++)
        {
            auto it = _obstacleMeasurements.find(itCandidate->identifier);

            // If not found, add it and feed to object discriminator
            if(it == _obstacleMeasurements.end())
            {
                _obstacleMeasurements[itCandidate->identifier] = (*itCandidate);
                _obstacleDiscriminator->addMeasurement(*itCandidate);
            }
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

void obstacleAdministrator::getLocalObstacleMeasurements(std::vector<obstacleMeasurement> &measurements)
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
    // store so it can be used to filter out fake obstacles (shadows of ourselves)
    _ownPos = ownLocation;
}

void obstacleAdministrator::notifyTeamMembers(std::vector<robotClass_t> const &teamMembers)
{
    // store so it can be used to filter out fake obstacles (shadows of ourselves)
    _teamMembers = teamMembers;
}

void obstacleAdministrator::performCalculation(rtime const timeNow)
{
    std::ostringstream ss;
    ss << "#overrule=" << _overruledObstacles.size();
    TRACE_FUNCTION(ss.str().c_str());
    try
    {
        cleanUpTimedOutObstacleMeasurements(timeNow);

        // in case of simulation, _overruledObstacles is used instead of _obstacleDiscriminator
        if (_overruledObstacles.size() == 0)
        {
            // real mode, not simulation - calculate
            std::vector<robotClass_t> teamMembersInclSelf = _teamMembers;
            teamMembersInclSelf.push_back(_ownPos);
            _obstacleDiscriminator->performCalculation(timeNow, teamMembersInclSelf);
            // get result, cache for getObstacles()
            _resultObstacles = _obstacleDiscriminator->getObstacles();
        }
        else
        {
            // send the simulated obstacles(s) to visualizer
            std::vector<obstacleClass_t> obstacles;
            for(auto it = _overruledObstacles.begin(); it != _overruledObstacles.end(); it++)
            {
                for(auto it2 = it->second.begin(); it2 != it->second.end(); it2++)
                {
                    obstacles.push_back(*it2);
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

void obstacleAdministrator::cleanUpTimedOutObstacleMeasurements(rtime const timeNow)
{
    TRACE_FUNCTION("");
    size_t origSize = _obstacleMeasurements.size();
    try
    {
        double maxTimeToLive = administrationConfigurator::getInstance().getObstacleTimeToLive();

        for(auto i = _obstacleMeasurements.begin(); i != _obstacleMeasurements.end();)
        {
            double time_diff = double(timeNow - i->second.timestamp);
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
                double time_diff = double(timeNow - it->second.at(0).getTimestamp());
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
    std::ostringstream ss;
    ss << "origSize=" << origSize << "; newSize=" << _obstacleMeasurements.size();
    TRACE(ss.str().c_str());
}

void obstacleAdministrator::fillDiagnostics(diagWorldModel &diagnostics)
{
    TRACE_FUNCTION("");
    diagnostics.shared.numObstacleTrackers = _obstacleDiscriminator->numTrackers();

    _obstacleDiscriminator->fillDiagnostics(diagnostics);
}

