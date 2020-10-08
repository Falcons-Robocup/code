 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotLocalization.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#include <numeric>

#include "int/algorithms/robotLocalization.hpp"

#include "falconsCommon.hpp"
#include "cEnvironmentField.hpp" // field dimensions
#include "cDiagnostics.hpp"
#include "position2d.hpp"
#include "tracing.hpp"

#define TIMESTAMP_UNINITIALIZED (-1)
#define NO_TRACKER_ID (-1)

/* requirements:
 *
 *  * driving on encoders is supported indefinitely
 *  * data is fed by robotAdministrator, before calculation is called
 *  * orientation is chosen when going inplay, facing forwards
 *
 * design choices:
 *  * all input data and initialization events are traced, for analysis afterwards and reproducability / testability
 *  * vision trackers are without orientation, so we do not need to maintain duplicate lists
 *  * timestamps are parameters, so test suite can run at 'infinite' speed
 *  * one actual position is maintained
 *
 * the main update sequence of 1 tick (typically @30Hz):
 *  * processMotorDisplacements
 *    * transform RCS to FCS
 *    * update current position
 *  * distributeVisionMeasurements
 *    * assign each vision candidate to a tracker, create new if needed
 *  * determine bestTrackerId
 *    * use qualitative / time-depending criteria to evaluate which tracker 'wins'
 *  * applyBestCameraMeasurement
 *    * only if the best tracker was stimulated at this tick:
 *      * match it with current position (lowest penalty)
 *      * if switching between trackers (rare instances)
 *        * take over position unweighted from the tracker
 *      * else (regular update)
 *        * camera latency correction (~ 100ms)
 *        * weighted position update (~ 10%)
 *  * determineIsValidAndOrient
 *    * invalid until 10 seconds (settling time), after which we orient forwards
 *    * stay valid even if all vision trackers die (trackerTimeout), to allow positioning for throw-in
 *  * cleanup buffers and dead trackers
 *
 *
 *  ROADMAP / future improvements
 *  * warning when deprived of vision for > 10 seconds, 
 *    error when this happens inside the field because this typically suggests the vision tube is skewed
 *  * use average confidence in vision tracker score calculation
 *  * use encoder displacement to update vision trackers, to avoid
 *    that a good vision match at high speed leads to a new tracker
 *  * coding: merge tracker getPosition & matchPosition - consider moving outside of tracker?
 *  * make errorRadianToMeter position-dependent, to reduce flip sensitivity at center of the field
 *  * reconsider core coding concept of the update sequence: always drive on encoders, vision acts as an interrupt?
 *
 **/

/*** begin public functions ***/

robotLocalization::robotLocalization(const WorldModelConfig* wmConfig)
{
    TRACE("creating robotLocalization");
    _wmConfig = wmConfig;
    _lastTrackerId = NO_TRACKER_ID;
    _firstVisionTimestamp = TIMESTAMP_UNINITIALIZED;
    _currentTimestamp = TIMESTAMP_UNINITIALIZED;
    _cameraMeasurementsBuffer.clear();
    _motorDisplacementsBuffer.clear();
    _motorVelocitiesBuffer.clear();
    _trackers.clear();
    _isValid = 0;
    _initializedInTTA = false;
    _currentPosVel.setCoordinateType(coordinateType::FIELD_COORDS); 
    _currentPosVel.setRobotID(getRobotNumber());
    initializeTTA();
    _lastMovingTimestamp = 0.0;
    TRACE("robotLocalization created");
}

robotLocalization::~robotLocalization()
/*
 *    If it looks like chicken, tastes like chicken,
 *    and feels like chicken but Chuck Norris says its beef, then it's beef.
 */
{

}

void robotLocalization::initializeTTA()
{
    // get TTA configuration
    areaInfo tta = cEnvironmentField::getInstance().getTTAarea();
    if (fabs(tta.R.getMaxY()) > 0.1)
    {
        poiInfo poiTTA = cEnvironmentField::getInstance().getFieldPOI(P_TTA_3); // typically center
        // don't need to fine-tune initial positions, we do not know the roles yet
        // lock & obstacle avoidance should take care of avoiding collisions
        _initializedInTTA = true;
        // face forward to properly set playing direction upon getting first vision loc
        _currentPosVel.setCoordinates(poiTTA.x, poiTTA.x, M_PI * 0.5);
    }
}

void robotLocalization::addVisionMeasurement(robotMeasurementClass_t const &measurement)
{
    traceVisionMeasurement(measurement);
    float minimumConfidence = _wmConfig->getConfiguration().localization.minimumConfidence;
    if (measurement.getConfidence() > minimumConfidence)
    {
        _cameraMeasurementsBuffer.push_back(measurement);
        if (_firstVisionTimestamp == TIMESTAMP_UNINITIALIZED)
        {
            TRACE_INFO("received first vision measurement");
            _firstVisionTimestamp = double(measurement.getTimestamp());
            // since we do not know better (0,0,0 initial position is arbitrary),
            // this measurement is stored directly in currentPosition
            _currentPosVel.setCoordinates(measurement.getX(), measurement.getY(), measurement.getTheta());
            orientTowardsOpponentGoal();
        }
    }
}

void robotLocalization::addDisplacement(robotDisplacementClass_t const &displacement)
{
    try
    {
        traceDisplacement(displacement);
        switch(displacement.getDisplacementSource())
        {
        	case displacementType::MOTORS:
        	{
        		_motorDisplacementsBuffer.push_back(displacement);
        		break;
        	}

        	case displacementType::IMU:
        	{
                                // Not processed at the moment
        		break;
        	}

        	default:
        	{
        		TRACE_ERROR("Received displacement with INVALID displacement source");
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
}


void robotLocalization::addVelocity(robotVelocityClass_t const &velocity)
{
    try
    {
        traceVelocity(velocity);
        switch(velocity.getDisplacementSource())
        {
            case displacementType::MOTORS:
            {
                _motorVelocitiesBuffer.push_back(velocity);
                break;
            }

            case displacementType::IMU:
            {
                                // Not processed at the moment
                break;
            }

            default:
            {
                TRACE_ERROR("Received displacement with INVALID displacement source");
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
}

void robotLocalization::calculatePositionAndVelocity(double timestampNow)
{
    TRACE_FUNCTION("");
    try
    {
        //TRACE(">");

        // explicitly store timestamp in tracing
        _currentTimestamp = timestampNow;
        _currentPosVel.setTimestamp(_currentTimestamp);
        traceCalculatePositionAndVelocity();

        // process the buffer of encoder displacement samples
        // return accumulated delta in FCS; also store velocity
        Position2D deltaDisplacementPos = processMotorDisplacementsAndVelocities();
        checkIfMoving(deltaDisplacementPos);
        
        // update all vision trackers with encoder displacement
        //TODO fix this filter . // updateVisionTrackersDisplacement(deltaDisplacementPos);
        
        // process the buffer of vision localization measurements
        // assign each to a tracker
        distributeVisionMeasurements();
        
        // choose best tracker
        int bestTrackerId = getBestTrackerId();
        
        // apply the camera measurement with some weight factor
        // generate a warning when switching from one tracker to another
        applyBestCameraMeasurement(bestTrackerId);

        // determine isValid, log state change
        // when switching from invalid to valid, re-orient 
        determineIsValidAndOrient();

        // diagnostics
        gatherDiagnostics(bestTrackerId);
        
        // cleanup (dead trackers etc)
        cleanup();

        //TRACE("<");
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

robotClass_t robotLocalization::getRobotPositionAndVelocity(double timestamp) const
{
    TRACE_FUNCTION("");
    robotClass_t result = _currentPosVel;
    if (timestamp > 0.01) // abuse zero as regression case
    {
        // latency correction, used to tag ball- and obstacle measurements with camera position
        Position2D offset = getEncoderUpdateSince(timestamp);
        Position2D uncorrected(result.getX(), result.getY(), result.getTheta());
        Position2D corrected = uncorrected - offset;
        //TRACE("LATENCY corrected phi from %6.2f to %6.2f", uncorrected.phi, corrected.phi);
        result.setCoordinates(corrected.x, corrected.y, corrected.phi);
    }
    return result;
}

localizationDiagnostics_t robotLocalization::getDiagnostics() const
{
    return _diagData;
}

void robotLocalization::triggerInitialization(double timestampNow)
{
    _trackers.clear();
    _lastTrackerId = NO_TRACKER_ID;
    _firstVisionTimestamp = TIMESTAMP_UNINITIALIZED;
    _isValid = false;
    TRACE_INFO("triggering localization init");
    // IO tracing for analysis, debugging and development
    // see also trace2testIO.sh in testsuite
    //TRACE("INITIALIZE      %16.6f", timestampNow); // TODO PTRACE
}

bool robotLocalization::isValid()
{
    return _isValid;
}

/*** end public functions ***/

/*** begin calculate sequence functions ***/

Position2D robotLocalization::processMotorDisplacementsAndVelocities()
/*!
 * \brief Apply the motor displacement values to current position.
 * Do the transformation per tick instead of lumping them first, 
 * otherwise we would accumulate errors while moving.
 *
 * Note: we assume garbage- and slip filtering is done at the source (peripheralsInterface/motorBoards).
 */
{
    // get current position as Position2D struct
    Position2D previousPos(_currentPosVel.getX(), _currentPosVel.getY(), _currentPosVel.getTheta());
    Position2D currentPos = previousPos;
    float vX = 0.0;
    float vY = 0.0;
    float vPhi = 0.0;

    // work through the vector of displacement values
    for (auto it = _motorDisplacementsBuffer.begin(); it != _motorDisplacementsBuffer.end(); ++it)
    {
        if (it->getCoordinateType() == coordinateType::ROBOT_COORDS)
        {
            // (ab)use velocity transformations
            Velocity2D displacementRcs(it->getdX(), it->getdY(), it->getdTheta());
            Velocity2D displacementFcs = displacementRcs.transform_rcs2fcs(currentPos);
            // store FCS update in history buffer, for latency correction
            auto displacementMeasurementFcs = *it;
            displacementMeasurementFcs.setDeltaPosition(displacementFcs.x, displacementFcs.y, displacementFcs.phi);
            // fix timestamp: in RTDBInputAdapter it is set to rtime::now, but this is not compatible with stimulator
            displacementMeasurementFcs.setTimestamp(_currentTimestamp);
            _motorDisplacementsHistory.push_back(displacementMeasurementFcs);
            // back to position
            currentPos += Position2D(displacementFcs.x, displacementFcs.y, displacementFcs.phi);
        }
        else if (it->getCoordinateType() == coordinateType::FIELD_COORDS)
        {
            Position2D displacementFcs(it->getdX(), it->getdY(), it->getdTheta());
            currentPos += displacementFcs;
        }
        else
        {
            TRACE_ERROR("got a displacement with invalid coordinate type");
            return Position2D();
        }
    }

    for (auto it = _motorVelocitiesBuffer.begin(); it != _motorVelocitiesBuffer.end(); ++it)
    {
        // take the average of last couple of encoder measurements (TODO: better to take the last, or are they noisy?)
        vX += it->getvX() / (float) _motorVelocitiesBuffer.size();
        vY += it->getvY() / (float) _motorVelocitiesBuffer.size();
        vPhi += it->getvTheta() / (float) _motorVelocitiesBuffer.size();
    }

    // transform speed
    Velocity2D velocityResult(vX, vY, vPhi);
    velocityResult = velocityResult.transform_rcs2fcs(currentPos);

    // store resulting speed and updated position
    _currentPosVel.setVelocities(velocityResult.x, velocityResult.y, velocityResult.phi);
    _currentPosVel.setCoordinates(currentPos.x, currentPos.y, currentPos.phi);

    // return delta
    Position2D deltaPos = currentPos - previousPos;
    return deltaPos;
}

void robotLocalization::updateVisionTrackersDisplacement(Position2D const &deltaDisplacementPos)
/*!
 * \brief Update all vision trackers with encoder displacement.
 */
{
    for (auto it = _trackers.begin(); it != _trackers.end(); ++it)
    {
        it->second.updateExpectedPosition(deltaDisplacementPos); // TODO consider orientation
    }
}

void robotLocalization::distributeVisionMeasurements()
/*!
 * \brief Process the buffer of vision localization measurements,
 * assigning each to a tracker.
 * Camera measurement score is basically a penalty, higher as the position delta w.r.t. tracker increases.
 * Note that this is opposite from relative tracker score, where higher is better (1.0 being perfect).
 */
{
    // get configuration values
    float minimumConfidence = _wmConfig->getConfiguration().localization.minimumConfidence;
    float trackerScoreAcceptanceThreshold = _wmConfig->getConfiguration().localization.trackerScoreAcceptanceThreshold;

    // work through buffer
    //TRACE("#measurements=%d #trackers=%d", (int)_cameraMeasurementsBuffer.size(), (int)_trackers.size());
    // TODO: if multiple candidates are assigned to the same tracker, then the latest one 'wins' in terms of timestamp & position result. I don't think this is a problem in practice, but still, it could be improved. See testVisionSelect
    for (auto itMeas = _cameraMeasurementsBuffer.begin(); itMeas != _cameraMeasurementsBuffer.end(); ++itMeas)
    {
        // only consider 'good' vision candidates
        if (itMeas->getConfidence() > minimumConfidence)
        {
            // note: symmetry is handled within localizationTracker
            // choose tracker to assign to
            int bestTrackerId = NO_TRACKER_ID;
            float bestScore = 9999;
            for (auto itTracker = _trackers.begin(); itTracker != _trackers.end(); ++itTracker)
            {
                float score = itTracker->second.getCameraMeasurementScore(*itMeas);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestTrackerId = itTracker->first;
                }
            }
            // assign or make new?
            if (bestScore < trackerScoreAcceptanceThreshold)
            {
                TRACE("stimulated tracker id=%d, score=%6.2f", bestTrackerId, bestScore);
                _trackers.at(bestTrackerId).feedMeasurement(*itMeas, _currentTimestamp);
            }
            else
            {
                // request new id
                int newTrackerId = localizationTracker::requestId();
                TRACE("created new tracker id=%d, score was %6.2f", newTrackerId, bestScore);

                // create and feed
                _trackers.insert( {newTrackerId, localizationTracker(_wmConfig)} );
                _trackers.at(newTrackerId).setId(newTrackerId);
                _trackers.at(newTrackerId).feedMeasurement(*itMeas, _currentTimestamp);
            }
        }
    }
}
        
int robotLocalization::getBestTrackerId()
/*!
 * \brief Choose best tracker.
 * Tracker score is the product of several sub scores. Values range in [0.0, 1.0], higher is better.
 * Note that this is the opposite from measurement score, where it is treated as a penalty (higher is worse).
 */
{
    int result = NO_TRACKER_ID;
    float bestScore = -1;
    for (auto it = _trackers.begin(); it != _trackers.end(); ++it)
    {
        //TRACE("n=%d  it->first=%d  it->second.getId()=%d", _trackers.size(), it->first, it->second.getId());
        assert(it->first == it->second.getId());
        float score = it->second.getTrackerScore(_currentTimestamp);
        if (score > bestScore)
        {
            bestScore = score;
            result = it->first;
        }
    }
    return result;
}

void robotLocalization::checkIfMoving(Position2D const &deltaDisplacementPos)
{
    float xyTolerance = 1e-3;
    float phiTolerance = 1e-4;
    if ((std::max(fabs(deltaDisplacementPos.x), fabs(deltaDisplacementPos.y)) < xyTolerance)
        && (fabs(deltaDisplacementPos.phi) < phiTolerance))
    {
        ;// robot stands still
    }
    else
    {
        // robot is moving
        _lastMovingTimestamp = _currentTimestamp;
    }
}

float robotLocalization::calculateWeightFactor()
{
    float w = _wmConfig->getConfiguration().localization.visionOwnWeightFactor;
    return w; // disable for now, vision phi noise is quite good already, this only adds complexity in tuning & analysis
    
    // reduce the influence of vision over time, but only if robot is standing still
    float t = timeStandingStill();
    float settleTimeStart = 1.0; // when to start reducing the weight factor
    float settleTimeEnd = 3.0; // when to arrive at zero 
    // TODO make both parameters configurable 
    if (t > settleTimeStart)
    {
        return w * std::max(0.0, 1.0 - ((t - settleTimeStart) / (settleTimeEnd - settleTimeStart)));
    }
    return w;
}

void robotLocalization::applyBestCameraMeasurement(int bestTrackerId)
/*!
 * \brief Apply the best camera measurement with some weight factor.
 * Generate a warning when switching from one tracker to another.
 * Latency correction is performed by 'playback' of encoder displacements since last camera measurement.
 */
{
    // need at least one tracker
    if (bestTrackerId == NO_TRACKER_ID)
    {
        return;
    }
    // it needs to have been stimulated at this tick
    if (fabs(_trackers.at(bestTrackerId).getLastPokeTimestamp() - _currentTimestamp) > 0.001)
    {
        return;
    }
    // get current position, initialize result and get configurables
    Position2D currentPosition(_currentPosVel.getX(), _currentPosVel.getY(), _currentPosVel.getTheta());
    //TRACE("currentPosition = (%6.2f,%6.2f,%6.2f)", currentPosition.x, currentPosition.y, currentPosition.phi);
    Position2D newPosition;
    // get best camera measurement, match it with current position
    _trackers.at(bestTrackerId).matchPosition(currentPosition);
    Position2D visionPosition = _trackers.at(bestTrackerId).getPosition();
    //TRACE(" visionPosition = (%6.2f,%6.2f,%6.2f)", visionPosition.x, visionPosition.y, visionPosition.phi);
    // check if tracker was switched
    if (bestTrackerId != _lastTrackerId)
    {
        //TRACE("switching from tracker %d to tracker %d", _lastTrackerId, bestTrackerId);
        if (_lastTrackerId != NO_TRACKER_ID)
        {
            TRACE_WARNING("switching from tracker %d to tracker %d", _lastTrackerId, bestTrackerId);
        }
        // take over vision position unweighted from the tracker, 
        // choose its best symmetrical score to prevent flips
        _trackers.at(bestTrackerId).matchPosition(currentPosition);
        newPosition = _trackers.at(bestTrackerId).getPosition();
        _lastTrackerId = bestTrackerId;
    }
    else // weighted update
    {
        // determine encoder update since vision timestamp
        Position2D encoderUpdate = getEncoderUpdateSince(_trackers.at(bestTrackerId).getLastVisionTimestamp());
        //TRACE("  encoderUpdate = (%6.2f,%6.2f,%6.2f)", encoderUpdate.x, encoderUpdate.y, encoderUpdate.phi);
        // determine the most accurate position at the moment of the vision measurement
        Position2D playbackPosition = currentPosition - encoderUpdate;
        //TRACE("playbackPosition= (%6.2f,%6.2f,%6.2f)", playbackPosition.x, playbackPosition.y, playbackPosition.phi);
        // calculate delta, be careful with angle update
        Position2D deltaPosition = visionPosition - playbackPosition;
        deltaPosition.phi = project_angle_mpi_pi(visionPosition.phi - playbackPosition.phi);
        // apply delta with weight factor
        float weightFactor = calculateWeightFactor();
        //TRACE("  deltaPosition = (%6.2f,%6.2f,%6.2f)", deltaPosition.x, deltaPosition.y, deltaPosition.phi);
        newPosition = deltaPosition * weightFactor + currentPosition;
    }
    //TRACE("    newPosition = (%6.2f,%6.2f,%6.2f)", newPosition.x, newPosition.y, newPosition.phi);
    // store new position
    _currentPosVel.setCoordinates(newPosition.x, newPosition.y, newPosition.phi);
    _visStabList.push_front(newPosition);
}

void robotLocalization::determineIsValidAndOrient()
/*!
 * \brief Determine isValid boolean. Log state change as INFO event.
 * Additionally, when switching from invalid to valid, re-orient playing forwards.
 *
 * Valid criteria:
 *  - since initialization or going inplay, any 'good' camera measurement must have arrived from vision
 *  - additionally, its age must exceed some threshold ('settling time')
 * 
 * Reasons for this: it could be the case that robot goes into the field, but vision is still stuck in a bad
 * loc. It needs some time to settle, after which this algorithm re-initializes.
 *
 * Once localization is valid, it will never switch back to invalid, so we 'can' drive on encoders indefinitely.
 * Only switching to OUTOFPLAY will make localization invalid, but that can also be handled by destructing this class.
 * This is to allow robot to take a throwin while being deprived of vision localization.
 */
{
    //TRACE(">");
    // check timers
    bool currentIsValid = (gotVision() && timerSinceFirstVisionExpired());
    // allow blind initialization in TTA
    if (!_isValid && !currentIsValid && _initializedInTTA)
    {
        currentIsValid = true;
    }
    if (!_isValid)
    {
        // only if not already have a lock
        if (currentIsValid != _isValid)
        {
            TRACE_INFO("localization valid flag changed to %d", currentIsValid);
            if (currentIsValid)
            {
                orientTowardsOpponentGoal();
            }
        }
        _isValid = currentIsValid;
    }
    // IO tracing for analysis, debugging and development
    // see also trace2testIO.sh in testsuite
    //TRACE("RESULT          %16.6f %2d %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f", _currentTimestamp, _isValid,
    //      _currentPosVel.getX(), _currentPosVel.getY(), _currentPosVel.getTheta(),
    //      _currentPosVel.getVX(), _currentPosVel.getVY(), _currentPosVel.getVTheta());
    //PTRACE("LOC %2d %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f", _isValid,
    //      _currentPosVel.getX(), _currentPosVel.getY(), _currentPosVel.getTheta(),
    //      _currentPosVel.getVX(), _currentPosVel.getVY(), _currentPosVel.getVTheta());
    //TRACE("<");
}


void robotLocalization::cleanup()
/*!
 * \brief Cleanup buffers and such at the end of an iteration.
 */
{
    //TRACE(">");
    cleanupVisionTrackers();
    _cameraMeasurementsBuffer.clear();
    _motorDisplacementsBuffer.clear();
    _motorVelocitiesBuffer.clear();
    cleanupDisplacementHistory();
    //TRACE("<");
}

/*** end calculate sequence functions ***/

/*** begin other helper functions ***/

bool robotLocalization::gotVision()
{
    return timeSinceLastVision() < _wmConfig->getConfiguration().localization.trackerTimeout;
}

double robotLocalization::timeSinceLastVision()
{
    double result = 999;
    for (auto it = _trackers.begin(); it != _trackers.end(); ++it)
    {
        double elapsed = _currentTimestamp - it->second.getLastVisionTimestamp();
        result = fmin(result, elapsed);
    }
    return result;
}

double robotLocalization::timeStandingStill()
{
    if (_lastMovingTimestamp == TIMESTAMP_UNINITIALIZED)
    {
        return 0.0;
    }
    return _currentTimestamp - _lastMovingTimestamp;
}

bool robotLocalization::timerSinceFirstVisionExpired()
{
    if (_trackers.size() == 0)
    {
        return false;
    }
    // compare timestamp of first vision measurement since calibration with current timestamp
    double dt = _currentTimestamp - _firstVisionTimestamp;
    bool result = (dt >= _wmConfig->getConfiguration().localization.settlingTime);
    return result;
}

void robotLocalization::orientTowardsOpponentGoal()
/*!
 * \brief Set orientation of robot without compass
 * Reason: to be able to operate without the compass (like other teams do).
 * So during outofplay, vision and encoder data is processed as normal, (a) for diagnostics
 * and (b) to be accurate when going inplay.
 * When going inplay we either keep or flip position such that robot is pointing at +y in FCS.
 * That flip is done within orientTowardsOpponentGoal().
 * When robot gets stuck in its mirror location (which should happen <1 times per match --> KPI!),
 * the recovery procedure is simply:
 * - take outofplay
 * - wait until referee allows re-entry
 * - point robot into the right direction
 * - press inplay
 *
 * Roadmap: occasionally sync and trigger this flip automatically, somehow using info from friends & obstacles
 * (only if really needed, like during more 'physical' matches which involve pushing & collisions)
 */
{
    float currentTheta = project_angle_0_2pi(_currentPosVel.getTheta());
    //TRACE("currentTheta=%6.2f", currentTheta);
    if ((currentTheta > M_PI) && (currentTheta < 6.0)) // TODO: get rid of 2nd clause which basically is a bit of magic to deal with current simulation initialization, where initial position is (0,0,0) + possible numerical noise 
    {
        flipOrientation();
        TRACE_INFO("orientTowardsOpponentGoal: orientation has flipped");
    }
}

void robotLocalization::flipOrientation()
/*!
 * \brief Flips/rotates the posVel by 180 degrees around (0,0)
 * E.g. (10, 10, 0) => (-10, -10, 180)
 */
{
    float x = -_currentPosVel.getX();
    float y = -_currentPosVel.getY();
    float theta = M_PI + _currentPosVel.getTheta();
    _currentPosVel.setCoordinates(x, y, theta);
}

Position2D robotLocalization::getEncoderUpdateSince(double timestamp) const
/*!
 * \brief Get the most recent total encoder update. Used in camera latency correction.
 */
{
    Position2D result;
    // TODO: we make a marginal error, by ignoring cumulative position update due to rcs2fcs transformation
    for (auto it = _motorDisplacementsHistory.begin(); it != _motorDisplacementsHistory.end(); ++it)
    {
        if (double(it->getTimestamp()) > timestamp)
        {
            result += Position2D(it->getdX(), it->getdY(), it->getdTheta());
        }
    }
    return result;
}

void robotLocalization::cleanupVisionTrackers()
/*!
 * \brief Cleanup trackers which have not been poked for a while.
 */
{
    for (auto it = _trackers.begin(); it != _trackers.end(); )
    {
        if (it->second.isTimedOut(_currentTimestamp))
        {
            it = _trackers.erase(it);
        }
        else
        {
            it++;
        }
    }
}
        
void robotLocalization::cleanupDisplacementHistory()
/*!
 * \brief Cleanup motor displacement history.
 */
{
    float timeout = 1.0;
    _motorDisplacementsHistory.erase(std::remove_if(_motorDisplacementsHistory.begin(),
                                                    _motorDisplacementsHistory.end(),
                            [=](robotDisplacementClass_t& m) { return (_currentTimestamp - double(m.getTimestamp())) > timeout; }),
                            _motorDisplacementsHistory.end());    
}

/*** end other helper functions ***/

/*** begin diagnostics functions ***/

void robotLocalization::traceVisionMeasurement(const robotMeasurementClass_t measurement)
{
    // IO tracing for analysis, debugging and development
    // see also trace2testIO.sh in testsuite
    // PTRACE("VISION %16.6f %9.6f %9.6f %9.6f %6.3f %d", 
    //     double(measurement.getTimestamp()), measurement.getX(), measurement.getY(), measurement.getTheta(), 
    //     measurement.getConfidence(), (int)measurement.getCoordindateType());
}

void robotLocalization::traceDisplacement(const robotDisplacementClass_t displacement)
{
    // IO tracing for analysis, debugging and development
    // see also trace2testIO.sh in testsuite
    std::string coordinateTypeString = "ENCODER_RCS";
    if (displacement.getCoordinateType() == coordinateType::FIELD_COORDS)
    {
        coordinateTypeString = "ENCODER_FCS";
    }
    // TODO PTRACE
    //TRACE("%s     %16.6f %9.5f %9.5f %9.5f %d %d\n",
    //        coordinateTypeString.c_str(),
    //        double(displacement.getTimestamp()), displacement.getdX(), displacement.getdY(), displacement.getdTheta(), 
    //        displacement.getCoordinateType(), (int)displacement.getDisplacementSource());
}

void robotLocalization::traceVelocity(const robotVelocityClass_t velocity)
{
    // IO tracing for analysis, debugging and development
    // see also trace2testIO.sh in testsuite
    std::string coordinateTypeString = "ENCODER_RCS";
    if (velocity.getCoordinateType() == coordinateType::FIELD_COORDS)
    {
        coordinateTypeString = "ENCODER_FCS";
    }
    // TODO PTRACE
    //TRACE("%s     %16.6f %9.5f %9.5f %9.5f %d %d",
    //    coordinateTypeString.c_str(),
    //    double(displacement.getTimestamp()), displacement.getvX(), displacement.getvY(), displacement.getvTheta(),
    //    displacement.getCoordinateType(), displacement.getDisplacementSource());
}

void robotLocalization::traceCalculatePositionAndVelocity()
{
    // trace timestamp at which calculate was called, for playback/analysis afterwards
    TRACE("CALCULATE       %16.6f", _currentTimestamp);
}

float stddev(std::vector<float> const &v)
{
    // auxiliary function, not a data member ... TODO move to facilities
    float sum = std::accumulate(v.begin(), v.end(), 0.0);
    float mean = sum / v.size();
    std::vector<float> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](float x) { return x - mean; });
    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    return std::sqrt(sq_sum / v.size());
}

void robotLocalization::determineVisionStability()
/*!
 * \brief Calculate vision stability KPI's
 */
{
    //TRACE(">");
    // initialize
    _diagData.visionNoiseXY = 0.0;
    _diagData.visionNoisePhi = 0.0;
    int visionStabilityLength = _wmConfig->getConfiguration().localization.visionStabilityLength;
    // make sure list is long enough, otherwise no stats
    if ((int)_visStabList.size() < visionStabilityLength)
    {
        return;
    }
    // clear out oldest samples
    _visStabList.resize(visionStabilityLength);
    // do statistics w.r.t. running total current position and velocity
    std::vector<float> values_x;
    std::vector<float> values_y;
    std::vector<float> values_phi;
    for (auto it = _visStabList.begin(); it != _visStabList.end(); ++it)
    {
        values_x.push_back(it->x);
        values_y.push_back(it->y);
        values_phi.push_back(project_angle_mpi_pi(it->phi - _visStabList.begin()->phi));
    }
    // 3sigma values
    _diagData.visionNoiseXY = 3.0 * std::max(stddev(values_x), stddev(values_y));
    _diagData.visionNoisePhi = 3.0 * stddev(values_phi);
    //TRACE("<");
}

void robotLocalization::gatherDiagnostics(int bestTrackerId)
{
    //TRACE(">");
    determineVisionStability();
    _diagData.numVisionCandidates = _cameraMeasurementsBuffer.size();
    _diagData.numMotorDisplacementSamples = _motorDisplacementsBuffer.size();
    // _diagData.bestVisionCandidate not used anymore
    if (bestTrackerId != NO_TRACKER_ID)
    {
        _diagData.visionLocAge = _trackers.at(bestTrackerId).getLocAge(_currentTimestamp);
        // current position and velocity are logged on worldState level
        Position2D visionPosition = _trackers.at(bestTrackerId).getPosition();
        _diagData.bestVisionCandidate.x = visionPosition.x;
        _diagData.bestVisionCandidate.y = visionPosition.y;
        _diagData.bestVisionCandidate.Rz = visionPosition.phi;
        _diagData.confidence = _trackers.at(bestTrackerId).getVisionConfidence();
    }
    else
    {
        _diagData.confidence = 0.0;
    }
    _diagData.isValid = _isValid;
    // extra sanity checks
    robotClass_t posVel = getRobotPositionAndVelocity();
    float speedLimitXY = _wmConfig->getConfiguration().localization.speedLimitXY;
    float speedLimitPhi = _wmConfig->getConfiguration().localization.speedLimitPhi;
    float margin = 0.5;
    float positionLimitX = margin + 0.5 * cEnvironmentField::getInstance().getWidth();
    float positionLimitY = margin + 0.5 * cEnvironmentField::getInstance().getLength();
    if ((fabs(posVel.getX()) > positionLimitX) || (fabs(posVel.getY()) > positionLimitY))
    {
        // allow robot position in TTA
        if (!cEnvironmentField::getInstance().isPositionInArea(posVel.getX(), posVel.getY(), A_TTA))
        {
            TRACE_INFO_TIMEOUT(10.0, "position (%6.1f, %6.1f) out of bounds", posVel.getX(), posVel.getY());
        }
    }
    Velocity2D vel(posVel.getVX(), posVel.getVY(), 0);
    if ((vel.size() > speedLimitXY) || (fabs(posVel.getVTheta()) > speedLimitPhi))
    {
        TRACE_INFO_TIMEOUT(2.0, "speed is too large (vx=%6.1fm/s, vx=%6.1fm/s, vphi=%6.1frad/s)", vel.x, vel.y, posVel.getVTheta());
    }
    //TRACE("<");
}

/*** end diagnostics functions ***/

