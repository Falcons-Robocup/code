 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleTracker.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/obstacleTracker.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

size_t obstacleTracker::_staticTrackerID = 0;

obstacleTracker::obstacleTracker(const objectMeasurementCache &measurement, const WorldModelConfig& wmConfig)
    : _wmConfig(wmConfig)
/*!
 * \brief Top-level obstacle tracker that calculates the obstacle position
 *
 * This class functions as a shell for future algorithms to track the obstacle
 *
 */
{
    _obstacleMeasurements.clear();
    _obstacleMeasurements.push_back(measurement);
    _staticTrackerID++;
    _trackerID = _staticTrackerID;
    _lastObstacleResult.setId(_trackerID);

    // initialize obstacleResult so new measurements can be added w.r.t. its position
    Vector3D pos = _obstacleMeasurements.back().getPositionFcs();
    _lastObstacleResult.setCoordinates(pos.x, pos.y, 0.0);
    
    // configurables
    _tracker.setConfig(_wmConfig.getConfiguration().obstacleTracker.objectFit);
}

obstacleTracker::~obstacleTracker()
/*
 * Chuck Norris CAN find the end of a circle
 */
{
    _obstacleMeasurements.clear();
}

bool sortOnIncreasingTime(const objectMeasurementCache& objA, const objectMeasurementCache& objB)
{
    return objA.getObjectMeasurement().timestamp < objB.getObjectMeasurement().timestamp;
}

void obstacleTracker::addObstacleMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded)
{
    try
    {
    	/*
    	 * Here we either accept or reject measurement for this tracker object
    	 * Current method:
    	 *  - take position from latest solution
    	 *    (if not available yet, which happens a brief moment after creation, then take latest measurement)
    	 *  - check distance against threshold
    	 */
        Vector3D currentPos(_lastObstacleResult.getX(), _lastObstacleResult.getY(), 0.0);
        Vector3D newPos = measurement.getPositionFcs();
        float threshold = _wmConfig.getConfiguration().obstacleTracker.trackerXYTolerance;
        if (vectorsize(currentPos - newPos) < threshold)
    	{
    	    // invariant: measurements shall be sorted by time
    	    // optimization: only sort if needed, i.e. timestamp is not newer than the last one
    	    if (measurement.getObjectMeasurement().timestamp < _obstacleMeasurements.back().getObjectMeasurement().timestamp)
    	    {
        	    _obstacleMeasurements.push_back(measurement);
        	    std::sort(_obstacleMeasurements.begin(), _obstacleMeasurements.end(), sortOnIncreasingTime);
    	    }
    	    else
    	    {
        	    _obstacleMeasurements.push_back(measurement);
        	    // no sort required
    	    }
    	    measurementIsAdded = true;
        }
        else
        {
    	    measurementIsAdded = false;
        }
    }
    catch(std::exception &e)
    {
    	TRACE_ERROR("Caught exception: %s", e.what());
    	std::cout << "Caught exception: " << e.what() << std::endl;
    	throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void obstacleTracker::setConfidence(rtime const timeNow)
{
    _lastObstacleResult.setConfidence(1.0); 
    // for now, confidence is not applicable for obstacles
    
    // TODO: use number of contributing measurements? unless we get glitches which need filtering, we should be OK
    // although, when we start to use frontVision for obstacle detection as well, this will become an issue
    // first use case of that would be for the keeper during penalty, figuring out attackers position w.r.t. ball,
    // anticipating the direction of the shot
}

void obstacleTracker::performCalculation(rtime const timeNow)
{
    TRACE_FUNCTION("");
    try
    {
        // extrapolation or not? if latest measurement is too old for extrapolation,
        // then do not update, but reuse latest result
        double latestMeasurementTimestamp = _obstacleMeasurements.back().getObjectMeasurement().timestamp;
        if (double(timeNow) <= latestMeasurementTimestamp + _wmConfig.getConfiguration().obstacleTracker.extrapolationTimeout)
        {

            // entry point of the algorithm
            _tracker.solve(_obstacleMeasurements, timeNow, false);
            // store ball result
            _lastObstacleResult = obstacleClass_t(_tracker.getResult());
            TRACE("first pass solution for trackerID=%d pos=(%6.2f, %6.2f), vel=(%.2f, %6.2f)", _trackerID, _lastObstacleResult.getX(), _lastObstacleResult.getY(), _lastObstacleResult.getVX(), _lastObstacleResult.getVY());

            // unlike the ball, obstacles by definition have no Z component
            // the common objectTracker calculates it, so here we force to zero
            _lastObstacleResult.setCoordinates(_lastObstacleResult.getX(), _lastObstacleResult.getY(), 0.0);
            _lastObstacleResult.setVelocities(_lastObstacleResult.getVX(), _lastObstacleResult.getVY(), 0.0);

            // fallback checks on velocity: do not yield a speed vector if there are too few measurements (after outlier removal) 
            // also, if fit residual is large, then apparently the speed vector does not fit nicely --> fallback.
            // if needed, redo fit from scratch (so previously outliers are back on the table) but without speed tracking
            if (_wmConfig.getConfiguration().obstacleTracker.objectFit.speedFitOrder > 0)
            {
                bool redoWithoutSpeed = false;
                int N = _obstacleMeasurements.size();
                if (N - _tracker.getNumRemoved() < 10)
                {
                    redoWithoutSpeed = true;
                }
                if (_tracker.getFitResidual() > _wmConfig.getConfiguration().obstacleTracker.speedResidualThreshold)
                {
                    redoWithoutSpeed = true;
                }
                if (redoWithoutSpeed)
                {
                    // temporarily change fit config
                    // TODO poor way of changing an option... this needs refactoring
                    ConfigWorldModelObjectFit tmpConfig = _wmConfig.getConfiguration().obstacleTracker.objectFit;
                    tmpConfig.speedFitOrder = 0;
                    _tracker.setConfig(tmpConfig);
                    // solve
                    _tracker.solve(_obstacleMeasurements, timeNow, false);
                    _lastObstacleResult = obstacleClass_t(_tracker.getResult());
                    _lastObstacleResult.setCoordinates(_lastObstacleResult.getX(), _lastObstacleResult.getY(), 0.0);
                    _lastObstacleResult.setVelocities(0.0, 0.0, 0.0);
                    TRACE("fallback solution for trackerID=%d pos=(%6.2f, %6.2f), vel=0", _trackerID, _lastObstacleResult.getX(), _lastObstacleResult.getY());
                    // restore config
                    _tracker.setConfig(_wmConfig.getConfiguration().obstacleTracker.objectFit);
                }
            }
            
            /*
            // MVEH: disabled, excessive spam in visualizer for high speed warnings
            // extra checks on velocity
            // if too slow, then yield 0 speed
            // (probably not needed, as long as PathPlanning is robust for small speed vectors / does not "get scared" too much)
            // if too fast, then generate a warning because this could suggests poor measurements or bad algorithm/tuning or ...
            float speed = Vector2D(_lastObstacleResult.getVX(), _lastObstacleResult.getVY()).size();
            if (speed < _wmConfig.getConfiguration().obstacleTracker.speedMinSize)
            {
                // too slow
                _lastObstacleResult.setVelocities(0.0, 0.0, 0.0);
            }
            if (speed > _wmConfig.getConfiguration().obstacleTracker.speedMaxSize)
            {
                // too fast: warning
                int timeout = 3;
                TRACE_WARNING_TIMEOUT(timeout, "obstacle with high speed (%6.2fm/s)", speed);
            }
            */
            
            // store id
            _lastObstacleResult.setId(_trackerID);

            setConfidence(timeNow);
            _lastObstacleResult.setTimestamp(timeNow);
        }

        cleanUpTimedOutObstacleMeasurements(timeNow);
    }
    catch(std::exception &e)
    {
    	TRACE_ERROR("Caught exception: %s", e.what());
    	std::cout << "Caught exception: " << e.what() << std::endl;
    	throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool obstacleTracker::isTimedOut(rtime const timeNow)
{
    try
    {
    	bool retVal = false;

    	// State when tracker is not longer valid
    	// Current behavior: when list of measurements are empty
    	cleanUpTimedOutObstacleMeasurements(timeNow);
    	retVal = _obstacleMeasurements.empty();

    	return retVal;
    }
    catch(std::exception &e)
    {
    	TRACE_ERROR("Caught exception: %s", e.what());
    	std::cout << "Caught exception: " << e.what() << std::endl;
    	throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void obstacleTracker::checkFake(std::vector<robotClass_t> const &teamMembers)
{
    TRACE_FUNCTION("");
    // note: list of teammembers includes own location (see obstacleAdministrator.cpp)

    // use list of current teammember locations to check if obstacle is fake
    // store it as a property, so getters / diagnostics can make use of it
    try
    {
        // initialize
        _fake = false; // until proven otherwise
        float thresholdMember = _wmConfig.getConfiguration().obstacleTracker.filterXYmemberTolerance;
        Vector2D obstPos(_lastObstacleResult.getX(), _lastObstacleResult.getY());

        // find robot closest to obstacle
        float minDist = 1e9;
        int closestRobotId = 0;
        for (auto itRobot = teamMembers.begin(); itRobot != teamMembers.end(); ++itRobot)
        {
            Vector2D robotPos(itRobot->getX(), itRobot->getY());
            float dist = (obstPos - robotPos).size();
            //TRACE("obstacle id=%d at (%6.2f,%6.2f) has distance %6.2f to robot %d", _trackerID, obstPos.x, obstPos.y, dist, itRobot->getRobotID());
            if (dist < minDist)
            {
                minDist = dist;
                closestRobotId = itRobot->getRobotID();
            }
        }

        // check if obstacle is seen by closest robot
        if (minDist < thresholdMember)
        {
            // fake unless at least one contributing measurement by the robot
            bool tmpFake = true;
            for (auto itMeas = _obstacleMeasurements.begin(); itMeas != _obstacleMeasurements.end(); ++itMeas)
            {
                if (itMeas->getObjectMeasurement().identifier.robotID == closestRobotId)
                {
                    tmpFake = false;
                }
            }
            if (tmpFake == true)
            {
                TRACE("filtering fake obstacle id=%d which has distance %6.2f to robot %d", _trackerID, minDist, closestRobotId);
                _fake = true;
            }
        }

        // TODO use thresholdOwn to ignore own-robot stuff (cables, wifi antennae, lens cape, etc.), closeby?
        // no, this should be handled either by hardware or by vision export filtering
        // here we do not have enough information to robustly apply such filtering - it could be the case that we are pushed by other robot
        
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

obstacleClass_t obstacleTracker::getObstacle() const
{
    try
    {
    	return _lastObstacleResult;
    }
    catch(std::exception &e)
    {
    	TRACE_ERROR("Caught exception: %s", e.what());
    	std::cout << "Caught exception: " << e.what() << std::endl;
    	throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void obstacleTracker::cleanUpTimedOutObstacleMeasurements(rtime const timeNow)
{
    try
    {
    	auto trackerTimeout = _wmConfig.getConfiguration().obstacleTracker.trackerTimeout;

    	for(auto i = _obstacleMeasurements.begin(); i != _obstacleMeasurements.end(); )
    	{
    		double time_diff = double(timeNow - i->getObjectMeasurement().timestamp);
    		if(time_diff > trackerTimeout)
            {
                // Delete obstacle measurement
    			i = _obstacleMeasurements.erase(i);
            }
            else
            {
                ++i;
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

std::string obstacleTracker::toStr(rtime const tcurr)
{
    char buf[256];
    int N = (int)_obstacleMeasurements.size();
    assert(N > 0);
    double age = tcurr - _obstacleMeasurements.begin()->getObjectMeasurement().timestamp;
    double freshness = tcurr - _obstacleMeasurements.back().getObjectMeasurement().timestamp;
    int numRemoved = _tracker.getNumRemoved(); 
    float residual = _tracker.getFitResidual(); 
    sprintf(buf, "%4d %d %7.3f %7.3f %7.3f %7.3f %3d %3d %6.3f %6.3f %6.3f", 
            (int)_trackerID, _fake,
            _lastObstacleResult.getX(), _lastObstacleResult.getY(),
            _lastObstacleResult.getVX(), _lastObstacleResult.getVY(),
            N, numRemoved, residual, age, freshness);
    std::string result = buf;
    return result;
}


