 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleDiscriminator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/obstacleDiscriminator.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/configurators/obstacleTrackerConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"

obstacleDiscriminator::obstacleDiscriminator()
{
	_obstacleTrackers.clear();
	
	// configurables
    _config.xLimit = obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::
discriminatorLimitX);
    _config.yLimit = obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::
discriminatorLimitY);

}

obstacleDiscriminator::~obstacleDiscriminator()
/*
 *	Chuck Norris can kill two stones with one bird.
 */
{

}

int obstacleDiscriminator::numTrackers() const
{
    return _obstacleTrackers.size();
}

void obstacleDiscriminator::addMeasurement(const obstacleMeasurementType &measurement)
{
	try
	{
        // wrap the measurement, calculate obstacle (x,y,z) in FCS
        objectMeasurementCache obst(measurement);

        // check if measurement should be rejected based on field boundaries,
        // to limit the amount of useless trackers
        Vector3D pos = obst.getPositionFcs();
        if ((fabs(pos.x) > obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::
discriminatorLimitX)) 
            || (fabs(pos.y) > obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::
discriminatorLimitY)))
        {
            //TRACE("reject out of bounds obstacle pos.x=%6.2f  pos.y=%6.2f", pos.x, pos.y);
            return;
        }
	    
        /*
         * Find closest tracker, do not yet insert
         */
        //TRACE("numTrackers=%d", (int)_obstacleTrackers.size());
        float closestDistance = 999;
        auto closestTrackerIterator = _obstacleTrackers.end();
        for (auto it = _obstacleTrackers.begin(); it != _obstacleTrackers.end(); it++)
        {
            Vector3D trackerPos(it->getObstacle().getX(), it->getObstacle().getY(), 0);
            float distance = vectorsize(pos - trackerPos);
            //TRACE("distance=%6.2f", distance);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestTrackerIterator = it;
            }
        }
        //TRACE("closestDistance=%6.2f", closestDistance);

        /*
        * If there is any tracker, then try to insert on it
        * success depends on tracker settings
        */
        bool measurementInserted = false;
        size_t assignedTrackerId = 0;
        if (_obstacleTrackers.size() != 0)
        {
            closestTrackerIterator->addObstacleMeasurement(obst, measurementInserted);
            if (measurementInserted)
            {
                assignedTrackerId = closestTrackerIterator->getObstacle().getId();
            }
        }
        if ((_obstacleTrackers.size() == 0) || !measurementInserted)
        {
            /*
             * Create new trackers and add to vector
             */
            obstacleTracker newTracker(obst);
            _obstacleTrackers.push_back(newTracker);
            assignedTrackerId = newTracker.getObstacle().getId();
        }

        // trace raw measurements, for playback/analysis afterwards
	    TRACE("%2d  1  %5d  %16.6f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f", 
	          measurement.getID().robotID, (int)assignedTrackerId, measurement.getTimestamp(),
	          measurement.getCameraX(), measurement.getCameraY(), measurement.getCameraZ(), measurement.getCameraPhi(),
	          measurement.getAzimuth(), measurement.getElevation(), measurement.getRadius(), measurement.getConfidence());

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

std::vector<obstacleClass_t> obstacleDiscriminator::getObstacles() const
{
	try
	{
		std::vector<obstacleClass_t> obstacles;

		for(auto it = _obstacleTrackers.begin(); it != _obstacleTrackers.end(); it++)
		{
			obstacles.push_back(it->getObstacle());
		}

		return obstacles;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void obstacleDiscriminator::traceTrackers(const double tcurr)
{
    // for now: all trackers are taken into account, there is no 'good' selection performed
    int num_good = (int)_obstacleTrackers.size();
    // show all track
	for(auto it = _obstacleTrackers.begin(); it != _obstacleTrackers.end(); it++)
	{
        // get tracker details
        std::string trackerInfo = it->toStr(tcurr);
        TRACE("%16.6f %3d %2d %s", tcurr, _obstacleTrackers.size(), num_good, trackerInfo.c_str());
    }
}

void obstacleDiscriminator::performCalculation(const double timeNow)
{
	try
	{
	    TRACE("#trackers=%d", (int)_obstacleTrackers.size());
	    TRACE("cleaning up (t=%16.6f)", timeNow);
		removeTimedOutTrackers(timeNow);
	    TRACE("remaining #trackers=%d", (int)_obstacleTrackers.size());
	
		for(auto it = _obstacleTrackers.begin(); it != _obstacleTrackers.end(); it++)
		{
			it->performCalculation(timeNow);
		}
		
		traceTrackers(timeNow);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void obstacleDiscriminator::removeTimedOutTrackers(const double timeNow)
{
	try
	{
		for(auto it = _obstacleTrackers.begin(); it != _obstacleTrackers.end(); )
		{
			if(it->isTimedOut(timeNow))
			{
				it = _obstacleTrackers.erase(it);
			}
			else
			{
			    it++;
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

