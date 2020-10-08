 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotLocalization.hpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTLOCALIZATION_HPP_
#define ROBOTLOCALIZATION_HPP_

#include <vector>
#include <deque>
#include <map>

#include "position2d.hpp"

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotVelocityType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/robot/localizationDiagnosticsType.hpp"
#include "int/algorithms/localizationTracker.hpp"


// Processes measurement and displacement information into an updated current (estimated) robot location
// A number calls to addVisionMeasurement/addDisplacement need to be (eventually) followed
// by a call to calculatePositionAndVelocity. After that, getRobotPositionAndVelocity may be called to retrieve the currently calculated localization.
class robotLocalization
{
    public:
        robotLocalization(const WorldModelConfig* wmConfig);
        ~robotLocalization();

        // add-calculate-get main contract
        // inputs and calculates explicitly get timestamp, for testability
        void addVisionMeasurement(robotMeasurementClass_t const &measurement);
        void addDisplacement(robotDisplacementClass_t const &displacement);
        void addVelocity(robotVelocityClass_t const &velocity);
        void calculatePositionAndVelocity(double timestampNow);
        robotClass_t getRobotPositionAndVelocity(double timestamp = 0.0) const;
        localizationDiagnostics_t getDiagnostics() const; // details
        
        // inplay button state changes
        void triggerInitialization(double timestampNow); // called when going inplay, triggers a time to settle vision
        bool isValid(); // false until a good vision measurement has arrived; will be reset by triggerInitialization

    private:
    
        /*** begin calculate sequence functions ***/
        
        // process the buffer of encoder displacement samples
        // return accumulated delta in FCS; also store velocity
        Position2D processMotorDisplacementsAndVelocities();
        
        // update all vision trackers with encoder displacement
        void updateVisionTrackersDisplacement(Position2D const &deltaDisplacementPos); 
        
        // process the buffer of vision localization measurements
        // assign each to a tracker
        void distributeVisionMeasurements();
        
        // choose best tracker
        int getBestTrackerId();

        // apply the camera measurement with some weight factor
        // generate a warning when switching from one tracker to another
        void applyBestCameraMeasurement(int bestTrackerId);

        // determine isValid, log state change
        // when switching from invalid to valid, re-orient 
        void determineIsValidAndOrient();

        // cleanup buffers
        void cleanup();
        
        /*** end calculate sequence functions ***/

    private:
            
        /*** begin other helper functions ***/
        
        bool gotVision(); // did we receive good vision data already?
        double timeSinceLastVision();
        double timeStandingStill();
        bool timerSinceFirstVisionExpired();
        float calculateWeightFactor();
        void checkIfMoving(Position2D const &deltaDisplacementPos);

        void orientTowardsOpponentGoal(); // Ensures that robot faces +y in FCS
        void flipOrientation();
        void initializeTTA();

        Position2D getEncoderUpdateSince(double timestamp) const;
        void cleanupVisionTrackers();
        void cleanupDisplacementHistory();
        
        /*** end other helper functions ***/

    private: 
    
        /*** begin data members ***/
        
        double _firstVisionTimestamp;
        double _currentTimestamp;
        double _lastMovingTimestamp;
        localizationDiagnostics_t _diagData;
        robotClass_t _currentPosVel;
        bool _isValid;
        bool _initializedInTTA = false;
        int _lastTrackerId;

        // buffers are flushed each heartbeat calculation
        std::vector<robotMeasurementClass_t> _cameraMeasurementsBuffer;
        std::vector<robotDisplacementClass_t> _motorDisplacementsBuffer;
        std::vector<robotVelocityClass_t> _motorVelocitiesBuffer;

        // history is needed for vision camera latency correction
        std::vector<robotDisplacementClass_t> _motorDisplacementsHistory;

        std::deque<Position2D> _visStabList;
        std::map<int, localizationTracker> _trackers;

        const WorldModelConfig* _wmConfig;

        /*** end data members ***/
        
    private:

        /*** begin diagnostics functions ***/

        void traceVisionMeasurement(const robotMeasurementClass_t measurement);
        void traceDisplacement(const robotDisplacementClass_t displacement);
        void traceVelocity(const robotVelocityClass_t velocity);
        void traceCalculatePositionAndVelocity();

        void determineVisionStability();
        void gatherDiagnostics(int bestTrackerId);

        /*** end diagnostics functions ***/

};

#endif /* ROBOTLOCALIZATION_HPP_ */

