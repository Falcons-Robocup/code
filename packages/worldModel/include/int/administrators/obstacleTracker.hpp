 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleTracker.hpp
 *
 *  Created on: Oct 5, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLETRACKER_HPP_
#define OBSTACLETRACKER_HPP_

#include <vector>

#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/algorithms/objectTracking.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/object/objectFitConfig.hpp"

struct obstacleTrackerConfig
{
    // triangulation and trajectory fit
    objectFitConfig fitConfig;
    float speedMinSize;
    float speedMaxSize;
    float speedResidualThreshold;

    // tracker administration
    float xyTolerance;
    double trackerTimeout;
    double extrapolationTimeout;
};


class obstacleTracker
{
public:
    obstacleTracker(const objectMeasurementCache &measurement);
    ~obstacleTracker();

    void addObstacleMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded);
    void performCalculation(rtime const timeNow);
    bool isTimedOut(rtime const timeNow);
    void checkFake(std::vector<robotClass_t> const &teamMembers);
    bool isFake() const { return _fake; };

    obstacleClass_t getObstacle() const;
    std::string toStr(rtime const tcurr);

private:
    std::vector<objectMeasurementCache> _obstacleMeasurements;
    objectTracker _tracker;
    obstacleClass_t _lastObstacleResult;
    bool _fake;
    static size_t _staticTrackerID;
    size_t _trackerID;
    obstacleTrackerConfig _config;

    void setConfidence(rtime const t);
    void cleanUpTimedOutObstacleMeasurements(rtime const timeNow);
};

#endif /* OBSTACLETRACKER_HPP_ */
