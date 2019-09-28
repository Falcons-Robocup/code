 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTracker.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLTRACKER_HPP_
#define BALLTRACKER_HPP_

#include <vector>
#include <string>

#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/types/ball/ballConfig.hpp"
#include "int/types/object/blackListType.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/algorithms/objectTracking.hpp"

#define MIN_BALL_HEIGHT 0.0
#define MIN_WARNING_CONFIDENCE 0.0
#define MIN_WARNING_HEIGHT -0.1

struct confidenceDetails
{
    int numCams;
    bool withOmni;
    float camScore;
    float ageScore;
    float measScore;
    float freshScore;
    float zScore;
    float vScore;
    float boundScore;
    float fitScore;
    float fitQuality;
};

class ballTracker
{
    public:
    	ballTracker(const objectMeasurementCache &measurement);
    	~ballTracker();

    	void addBallMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded);
    	void calculateBall(rtime const timeNow, bool ownBallsFirst = false);
    	bool isTimedOut(rtime const timeNow);
    	ballClass_t getBall() const;
    	bool isBlackListed() const;
    	std::string toStr(rtime const timeNow, bool details = false);
    	std::string xyzDetailsStr();
    	confidenceDetails getDetails() const { return _confDetails; }
        bool getOwnBallsFirst() const { return _ownBallsFirst; }
    	size_t numMeasurements() const { return _ballMeasurements.size(); }

        static void reset();
        void traceMeasurements() const;
    	std::vector<objectMeasurementCache> getBallMeasurements() { return _ballMeasurements; }
        
    private:
    	std::vector<objectMeasurementCache> _ballMeasurements;
    	ballClass_t _lastBallResult;
    	confidenceDetails _confDetails;
    	ballConfig _ballCfg;
    	objectTracker _tracker;
    	size_t _trackerID;
    	static size_t _staticTrackerID;
        blackListType _blackList;
        bool _ownBallsFirst;
        rtime _lastGroundTimeStamp;
        rtime _lastAirTimeStamp;
    	rtime _t0;
        rtime _tmin;
        rtime _tmax;

    	void selectOwnOmniMeasurements(std::vector<objectMeasurementCache> &measurements);
        void calculateConfidence(rtime const t);
    	void setBlackList(blackListType b);
    	void cleanUpTimedOutBallMeasurements(rtime const t);
};

#endif /* BALLTRACKER_HPP_ */
