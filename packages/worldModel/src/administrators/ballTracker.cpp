 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTracker.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballTracker.hpp"

#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp" // field dimensions
#include "tracing.hpp"


size_t ballTracker::_staticTrackerID = 0;

void ballTracker::reset()
{
    _staticTrackerID = 0;
}

ballTracker::ballTracker(const objectMeasurementCache &measurement, const WorldModelConfig* wmConfig)
    : _confDetails()
/*!
* \brief Top-level ball tracker that calculates the ball position
*
* This class functions as a shell for future algorithms to track the ball
*
*/
{
    _wmConfig = wmConfig;

    _ballMeasurements.push_back(measurement);
    _ownBallsFirst = false;
    _ownRobotId = getRobotNumber();
    _staticTrackerID++;
    _trackerID = _staticTrackerID;
    _lastBallResult.setId(_trackerID);

    // initialize ballResult so new measurements can be added w.r.t. its position
    Vector3D pos = _ballMeasurements.back().getPositionFcs();
    _lastBallResult.setCoordinates(pos.x, pos.y, pos.z);
    _lastBallResult.setVelocities(0.0, 0.0, 0.0);

    // store timestamp of creation for age calculation
    _t0 = measurement.getObjectMeasurement().timestamp;
    _lastBallResult.setTimestamp(_t0);
    _blackList = blackListType::UNKNOWN;
    _lastGroundTimeStamp = _t0;
    _lastAirTimeStamp = _t0;
    _tmin = _t0;
    _tmax = _t0;

    _tracker.setConfig(_wmConfig->getConfiguration().ballTracker.objectFit);
}

ballTracker::~ballTracker()
/*
* Chuck Norris doesn't wear a watch.
* He decides what time it is.
*/
{
    _ballMeasurements.clear();
}

bool sortOnIncreasingTime(const objectMeasurementCache& objA, const objectMeasurementCache& objB)
{
    return objA.getObjectMeasurement().timestamp < objB.getObjectMeasurement().timestamp;
}

bool ballTracker::checkDeltaAngles(const objectMeasurementCache &measurement, Vector3D const &expectedPos)
{
    // check if delta angles are small enough
    Position2D robotPos(measurement.getObjectMeasurement().cameraX, measurement.getObjectMeasurement().cameraY, measurement.getObjectMeasurement().cameraPhi);
    float camZ = measurement.getObjectMeasurement().cameraZ; 
    Vector3D measPos = measurement.getPositionFcs();
    // transform to robot perspective
    Position2D expectedFcs(expectedPos.x, expectedPos.y, 0.0);
    Position2D expectedRcs = Position2D(expectedFcs).transform_fcs2rcs(robotPos);
    Position2D measuredFcs(measPos.x, measPos.y, 0.0);
    Position2D measuredRcs = Position2D(measuredFcs).transform_fcs2rcs(robotPos);
    float expectedDistance = vectorsize(expectedRcs.xy());
    float measuredDistance = vectorsize(measuredRcs.xy());
    // prevent div by zero
    if (std::min(measuredDistance, expectedDistance) < 0.3)
    {   
        // if one of both is closeby, then we basically apply a proximity criterion: the other should also be closeby
        return std::max(measuredDistance, expectedDistance) < 0.3;
    }
    // angles
    float deltaAz = atan2(expectedRcs.x - robotPos.x, expectedRcs.y - robotPos.y) - atan2(measuredRcs.x - robotPos.x, measuredRcs.y - robotPos.y);
    float deltaEl = atan2(expectedPos.z - camZ, expectedDistance) - measurement.getObjectMeasurement().elevation;
    float threshold = _wmConfig->getConfiguration().ballTracker.solverTrackerConeTolerance;
    //tprintf("deltaAz=%.3f deltaEl=%.3f  %.3f %.3f %.3f %.3f", deltaAz, deltaEl, fabs(deltaAz), fabs(deltaEl), std::max(fabs(deltaAz), fabs(deltaEl)), threshold);
    return std::max(fabs(deltaAz), fabs(deltaEl)) < threshold;
}

void ballTracker::addBallMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded)
{
    //TRACE_FUNCTION("");
    try
    {
        /*
        * Here we either accept or reject measurement for this tracker object
        * Current method: 
        *  - take position from latest solution 
        *    (if not available yet, which happens a brief moment after creation, then take latest measurement)
        *  - calculate expected position by including speed vector
        *  - compare measurement with expected position, calculate angles from camera perspective
        *  - check angles (cone) against threshold; distance can hardly be trusted
        */

        Vector3D currentPos(_lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ());
        float dt = double(measurement.getObjectMeasurement().timestamp) - double(_lastBallResult.getTimestamp());
        Vector3D expectedPos = currentPos + dt * Vector3D(_lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ());
        if (checkDeltaAngles(measurement, expectedPos))
        {
            // as far as discriminator is concerned, this measurement will now be processed
            // in principle it is added and to be used, but it may also be the case that we here already reject it
            // in which case the measurement is not added to the vector, but flag measurementIsAdded is set
            // TODO: make this nicer (rename measurementIsAdded to measurementIsProcessed? filter elsewhere?)
            bool accept = true;
            auto camType = measurement.getObjectMeasurement().source;
            bool isHighCam = (camType == cameraEnum::FRONTCAMERA) || (camType == cameraEnum::MULTICAMERA);
            if (isHighCam)
            {
                if (measurement.getObjectMeasurement().identifier.robotID == getRobotNumber())
                {
                    accept = _wmConfig->getConfiguration().ballTracker.useOwnHighVision;
                }
                else
                {
                    accept = _wmConfig->getConfiguration().ballTracker.useFriendlyHighVision;
                }
            }
            else
            {
                _ballMeasurements.push_back(measurement);
                // a sort may be required
            }
            // update time range
            _tmin = std::min(_tmin, measurement.getObjectMeasurement().timestamp);
            _tmax = std::max(_tmax, measurement.getObjectMeasurement().timestamp);

            if (accept)
            {
                // update time range
                _tmin = std::min(_tmin, measurement.getObjectMeasurement().timestamp);
                _tmax = std::max(_tmax, measurement.getObjectMeasurement().timestamp);

                // floating ball detection
                // see also isBlackListed()
                Vector3D pos = measurement.getPositionFcs();
                bool airborne = (pos.z > _wmConfig->getConfiguration().ballTracker.blackListThresholdZ);
                rtime tMeas = measurement.getObjectMeasurement().timestamp;
                if (airborne)
                {
                    _lastAirTimeStamp = tMeas;
                    double floatingTime = tMeas - _lastGroundTimeStamp;
                    if (floatingTime > _wmConfig->getConfiguration().ballTracker.blackListFloatingDuration)
                    {
                        // what magic is this?! floating -> blacklist (probably a lamp, T-shirt, or a poster, ...)
                        setBlackList(blackListType::BLACK);
                    }
                }
                else
                {
                    _lastGroundTimeStamp = tMeas;
                    double groundTime = tMeas - _lastAirTimeStamp;
                    if (groundTime > _wmConfig->getConfiguration().ballTracker.blackListGroundDuration)
                    {
                        // this seems to be a real ball
                        setBlackList(blackListType::WHITE);
                    }
                }
                
                //auto m = measurement.getObjectMeasurement();
                //tprintf("added ball  id %5d  cam  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  pos  %9.5f  %9.5f  %9.5f",
                //    _trackerID, m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi, m.azimuth, m.elevation, m.radius, m.confidence, pos.x, pos.y, pos.z); 

            }
            
            //auto m = measurement.getObjectMeasurement();
            //tprintf("added ball  id %5d  cam  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  pos  %9.5f  %9.5f  %9.5f",
            //    _trackerID, m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi, m.azimuth, m.elevation, m.radius, m.confidence, pos.x, pos.y, pos.z); 

            // inform discriminator that this measurement is processed, stop iterating over trackers
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
    // invariant: measurements shall be sorted by time (needed by solver)
    // TODO: optimization: only sort if needed, i.e. timestamp is not newer than the last one
    // for now always sort
    std::sort(_ballMeasurements.begin(), _ballMeasurements.end(), sortOnIncreasingTime);
    ensureUniqueMeasurements(); // TODO integrate in above, and while we're at it, reconsider interface add/accept
    //traceMeasurements();
    //TRACE("measurementIsAdded=%d _ballMeasurements.size=%d", measurementIsAdded, (int)_ballMeasurements.size());
}

void ballTracker::ensureUniqueMeasurements()
{
    std::set<uniqueObjectID> seen;
    for (auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); /*intentionally empty*/ )
    {
        auto id = it->getObjectMeasurement().identifier;
        if (seen.count(id))
        {
            // Delete ball measurement
            it = _ballMeasurements.erase(it);
        }
        else
        {
            seen.insert(id);
            ++it;
        }
    }
};

void ballTracker::setBlackList(blackListType b)
{
    _blackList = b;
    return;
    if (b == blackListType::BLACK)
    {
        TRACE("tracker id=%d is now blacklisted", _trackerID);
    }
    else if (b == blackListType::WHITE)
    {
        TRACE("tracker id=%d is now whitelisted", _trackerID);
    }
}

void ballTracker::calculateConfidence(rtime const t)
{
    // ported implementation from fbt_tracker_confidence.m
    // confidence is determined by calculating several sub-scores, each between 0.0 (bad) and 1.0 (good)
    // final confidence is the product of each sub-score

    // iterate once through all measurements, to extract some values
    _confDetails.withOmni = false;
    std::set<uint8_t> usedCams;
    for (auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); ++it)
    {
        usedCams.insert(it->getObjectMeasurement().identifier.robotID);
        if (it->getObjectMeasurement().source == cameraEnum::OMNIVISION)
        {
            _confDetails.withOmni = true;
        }
    }

    // out-of-bounds score
    float edgeTolerance = 0.5f; // TODO reconfigurable
    float margin = 0.1f;
    float boundX = margin + 0.5f * cEnvironmentField::getInstance().getWidth();
    float boundY = margin + 0.5f * cEnvironmentField::getInstance().getLength();
    float boundScoreX = 1.0f - std::min(std::max((std::fabs(_lastBallResult.getX()) - boundX) / edgeTolerance, 0.0f), 1.0f);
    float boundScoreY = 1.0f - std::min(std::max((std::fabs(_lastBallResult.getY()) - boundY) / edgeTolerance, 0.0f), 1.0f);
    _confDetails.boundScore = boundScoreX * boundScoreY;

    // camera score (more is better, omnivision beats frontvision)
    _confDetails.numCams = 0;
    _confDetails.numCams = usedCams.size();
    float camScale = _wmConfig->getConfiguration().ballTracker.confidenceNumCams * (1 + _wmConfig->getConfiguration().ballTracker.confidenceOmniPref);
    _confDetails.camScore = std::min(_confDetails.numCams, _wmConfig->getConfiguration().ballTracker.confidenceNumCams) * (1 + (_confDetails.withOmni && _wmConfig->getConfiguration().ballTracker.confidenceOmniPref)) / camScale;

    // number of measurements (more is better)
    _confDetails.measScore = std::min(_tracker.getNumGood(), _wmConfig->getConfiguration().ballTracker.confidenceMeasLim) / (1.0 * _wmConfig->getConfiguration().ballTracker.confidenceMeasLim);

    // age (higher is better) and freshness (time since last measurement - lower is better)
    _confDetails.ageScore = std::min(1.0, double(t - _t0) / _wmConfig->getConfiguration().ballTracker.confidenceAgeLim);
    _confDetails.freshScore = 1.0 - std::max(0.0, (double(t - _tmax) - _wmConfig->getConfiguration().ballTracker.confidenceFreshLim) / (_wmConfig->getConfiguration().ballTracker.solverTrackerTimeout - _wmConfig->getConfiguration().ballTracker.confidenceFreshLim));

    // fit quality (lower is better)
    _confDetails.fitScore   = 1.0f - std::min(std::max((_confDetails.fitQuality - _wmConfig->getConfiguration().ballTracker.confidenceFitLim1) / (_wmConfig->getConfiguration().ballTracker.confidenceFitLim2 - _wmConfig->getConfiguration().ballTracker.confidenceFitLim1), 0.0f), 1.0f);

    // velocity (lower is better)
    Vector3D velocity(_lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ());
    _confDetails.vScore     = 1.0f - std::min(std::max((vectorsize(velocity) - _wmConfig->getConfiguration().ballTracker.confidenceVLim1) / (_wmConfig->getConfiguration().ballTracker.confidenceVLim2 - _wmConfig->getConfiguration().ballTracker.confidenceVLim1), 0.0f), 1.0f);

    // ball height (lower is better)
    _confDetails.zScore     = 1.0f - std::min(std::max((_lastBallResult.getZ() - _wmConfig->getConfiguration().ballTracker.confidenceZLim1) / (_wmConfig->getConfiguration().ballTracker.confidenceZLim2 - _wmConfig->getConfiguration().ballTracker.confidenceZLim1), 0.0f), 1.0f);

    // multiply all sub-scores to obtain final confidence
    float confidence = _confDetails.camScore * _confDetails.measScore * _confDetails.ageScore 
        * _confDetails.freshScore * _confDetails.vScore * _confDetails.zScore
        * _confDetails.fitScore * _confDetails.boundScore;

    // store
    _lastBallResult.setConfidence(confidence);
}

bool ballTracker::isBlackListed() const
{
    // there is a fundamental issue with detecting lob shots
    // suppose enemy is taking a free kick, where the receiving robot lobs immediately across the enemy which took the kick
    // TechUnited often does this 'tikkie takka'
    // then the goalie will see the ball first in the air and it needs to immediately respond to it (option A)
    // however, this will cause robots to also briefly respond to things like posters and lights, when they first are tracked
    // option B: more cauteous, only accept a ball after it has touched the ground
    // if friendly robots observe such a free kick from the side, this should still work out OK for the keeper

    // A. use this condition if we want aggressive but also glitchy behavior
    // any fresh flying tracker might be accepted, because it is not yet blacklisted
    if (!_wmConfig->getConfiguration().ballTracker.blackListDefault)
    {
        return (_blackList == blackListType::BLACK);
    }

    // B. use this condition if we want cauteous behavior
    // any fresh flying tracker is by default rejected, until it touches the ground, at which point it is whitelisted
    return (_blackList != blackListType::WHITE);
}

void ballTracker::calculateBall(rtime const timestampNow, bool ownBallsFirst)
{
    //TRACE_FUNCTION("");
    try
    {
        // entry point of the algorithm
        std::vector<objectMeasurementCache> measurements = _ballMeasurements;
        //TRACE("calculateBall %d measurements ownBallsFirst=%d", (int)_ballMeasurements.size(), ownBallsFirst);
        // pre-filter?
        if (ownBallsFirst)
        {
            selectOwnOmniMeasurements(measurements);
        }
        _ownBallsFirst = ownBallsFirst; // for diagnostics
        // containment for #3: high balls must always be sent from vision to worldModel
        // but worldModel is too sensitive, so for now, here we filter
        // (at least all data is logged in RDL so we can develop on it)
        selectLowMeasurements(measurements);
        // call solver, require at least one measurement present, otherwise we may not call solver
        if (measurements.size() == 0)
        {
            // return empty
            _lastBallResult = ballClass_t();
            _lastBallResult.setConfidence(0.0);
            _lastBallResult.setId(_trackerID);
            return;
        }
        // safe to call solver
        //TRACE("id=%d calling solver with %d measurements", (int)_trackerID, (int)measurements.size());
        _tracker.solve(measurements, timestampNow, true);
        // store ball result
        _lastBallResult = ballClass_t(_tracker.getResult());
        _lastBallResult.setId(_trackerID);
        // calculate confidence and extract some diagnostics values
        _confDetails.fitQuality = _tracker.getFitResidual();
        calculateConfidence(timestampNow);
        // floor clipping
        if (_lastBallResult.getZ() < MIN_BALL_HEIGHT)
        {
            // warn only if this ball had a high confidence and was significantly low in the ground
            if ((_lastBallResult.getConfidence() > MIN_WARNING_CONFIDENCE) && (_lastBallResult.getZ() < MIN_WARNING_HEIGHT))
            {
                TRACE_WARNING("clipping ball to floor (x=%6.2f, y=%6.2f, z=%6.2f, conf=%6.2f)", _lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ(), _lastBallResult.getConfidence());
                // TODO: give a penalty to the confidence, say 20%?
            }
            _lastBallResult.setCoordinates(_lastBallResult.getX(), _lastBallResult.getY(), 0.0);
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool ballTracker::isTimedOut(rtime const t)
{
    try
    {
        bool retVal = false;

        /*
        * State when tracker is not longer valid
        * Current behavior: when list of measurements are empty
        * or when last measurement exceeds tracker timeout
        */
        cleanUpTimedOutBallMeasurements(t);
        retVal = _ballMeasurements.empty();
        if (!retVal)
        {
            auto timeout = _wmConfig->getConfiguration().ballTracker.solverTrackerTimeout;
            double time_diff = t - _tmax;
            if (time_diff > timeout)
            {
                //tprintf("BALL TIMEOUT delta=%.6f t=%s %.6f _tmax=%s %.6f", time_diff, t.toStr().c_str(), double(t), _tmax.toStr().c_str(), double(_tmax));
                retVal = true;
            }
        }

        return retVal;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

ballClass_t ballTracker::getBall() const
{
    try
    {
        return _lastBallResult;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballTracker::selectLowMeasurements(std::vector<objectMeasurementCache> &measurements)
{
    try
    {
        for(auto i = measurements.begin(); i != measurements.end(); )
        {
            // elevation -0.12 matches with view distance of about 7m
            if (i->getObjectMeasurement().elevation > -0.12)
            {
                // Delete ball measurement
                i = measurements.erase(i);
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

void ballTracker::selectOwnOmniMeasurements(std::vector<objectMeasurementCache> &measurements)
{
    try
    {
        int ownRobotId = getRobotNumber();
        for(auto i = measurements.begin(); i != measurements.end(); )
        {
            if ((i->getObjectMeasurement().identifier.robotID != ownRobotId)
                || (i->getObjectMeasurement().source != cameraEnum::OMNIVISION))
            {
                // Delete ball measurement
                i = measurements.erase(i);
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

void ballTracker::cleanUpTimedOutBallMeasurements(rtime const t)
{
    try
    {
        auto bufferSize = _wmConfig->getConfiguration().ballTracker.solverTrackerBuffer;

        // remove old measurements
        for (auto i = _ballMeasurements.begin(); i != _ballMeasurements.end(); )
        {
            double time_diff = _tmax - i->getObjectMeasurement().timestamp;
            if (time_diff > bufferSize)
            {
                // Delete ball measurement
                i = _ballMeasurements.erase(i);
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

void ballTracker::traceMeasurements() const
{
    for (auto imeas = _ballMeasurements.begin(); imeas != _ballMeasurements.end(); ++imeas)
    {
        objectMeasurement m = imeas->getObjectMeasurement();
        Vector3D ballPos = imeas->getPositionFcs();
        TRACE("measurement:  id(%d/%6d)  t%16.6f  cx%9.5f  cy%9.5f  cz%9.5f  cr%9.5f  az%9.5f  el%9.5f   r%9.5f  cf%9.5f  bx%9.5f  by%9.5f  bz%9.5f",
            m.identifier.robotID, m.identifier.uniqueID,
            (double)m.timestamp, m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi,
            m.azimuth, m.elevation, m.radius, m.confidence,
            ballPos.x, ballPos.y, ballPos.z); 
    }
}

std::string ballTracker::toStr(rtime const tcurr, bool details)
{
    char buf[256];
    double age = tcurr - _t0;
    sprintf(buf, "%4d %4.2f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %5.1f %5d", 
            (int)_trackerID, _lastBallResult.getConfidence(), 
            _lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ(), 
            _lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ(), 
            age, _tracker.getNumGood());
    std::string result = buf;
    if (details)
    {
        sprintf(buf, " %1d %1d %5d %4d %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f",
                (int)_blackList, _ownBallsFirst,
                _confDetails.numCams, _confDetails.withOmni, 
                _confDetails.camScore, _confDetails.ageScore, _confDetails.measScore, _confDetails.freshScore,
                _confDetails.zScore, _confDetails.vScore, _confDetails.boundScore,
                _confDetails.fitScore, _confDetails.fitQuality);
        result += buf;
    }
    return result;
}

void ballTracker::makeDiagnostics(diagBallTracker &diag, rtime const &tcur)
{
    //TRACE_FUNCTION("id=%s", (int)getBall().getId()); // TODO support standard printf syntax in tracing, make its API more uniform
    //TRACE_FUNCTION("");
    float nominalFrequency = 30.0; // TODO common config?
    float trackingBufferDuration = _wmConfig->getConfiguration().ballTracker.solverTrackerBuffer;
    ballClass_t bc = getBall();
    ballResult br;
    br.position.x = bc.getX();
    br.position.y = bc.getY();
    br.position.z = bc.getZ();
    br.velocity.x = bc.getVX();
    br.velocity.y = bc.getVY();
    br.velocity.z = bc.getVZ();
    br.confidence = bc.getConfidence(); // often not calculated (-> remains 0.00)
    diag.result = br;
    diag.id = bc.getId();
    diag.ownGoodDataRate = 0.0;
    diag.outliersFraction = 0.0;
    diag.age = tcur - _t0;
    diag.freshness = tcur - _tmax;
    diag.obf = _ownBallsFirst;
    _tracker.makeDiagnostics(diag.measurements, diag.measurementClusters);
    // NOTE: it can happen that solver was not called (due to filters, or other performance reasons)
    // in that case, copy the measurements but leave clustering empty
    if (diag.measurements.size() == 0)
    {
        for (auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); ++it)
        {
            diagObjectMeasurement meas;
            meas.m = it->getObjectMeasurement();
            meas.used = true;
            diag.measurements.push_back(meas);
        }
    }
    auto bm = diag.measurements;
    // invert order: newest non-removed measurement first
    // (this is handy for analysis, when looking at azimuth during intercept)
    std::sort(begin(diag.measurements), end(diag.measurements), [](diagObjectMeasurement const& l, diagObjectMeasurement const& r)
    {
        if (l.used != r.used) return l.used > r.used;
        return l.m.timestamp > r.m.timestamp;
    });
    // count outliers
    for (auto it = bm.begin(); it != bm.end(); ++it)
    {
        if (it->used)
        {
            diag.ownGoodDataRate += (it->m.identifier.robotID == _ownRobotId);
        }
        else
        {
            diag.outliersFraction += 1.0;
        }
    }
    //TRACE("countOwnGood=%d trackingBufferDuration=%.1f ownGoodDataRate=%.2f", (int)diag.ownGoodDataRate, trackingBufferDuration, diag.ownGoodDataRate / (nominalFrequency / trackingBufferDuration));
    diag.ownGoodDataRate /= nominalFrequency / trackingBufferDuration;
    if (bm.size())
    {
        diag.outliersFraction /= bm.size();
    }
}

std::string ballTracker::xyzDetailsStr()
{
    return _tracker.getDetailsStr();
}

