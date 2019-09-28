 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationTracker.hpp
 *
 *  Created on: Jun 10, 2017
 *      Author: Jan Feitsma
 */

#ifndef LOCALIZATIONTRACKER_HPP_
#define LOCALIZATIONTRACKER_HPP_


#include <deque>
#include "position2d.hpp"

#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"

class localizationTracker
{
public:
    localizationTracker();
    ~localizationTracker();
    
    // setters
    void updateExpectedPosition(Position2D const &deltaDisplacementPos);
    void feedMeasurement(robotMeasurementClass_t const &measurement, double timestampNow);
    void setId(int id);
    void matchPosition(Position2D const &referencePosition);
    
    // getters
    float getCameraMeasurementScore(robotMeasurementClass_t const &measurement) const;
    float getTrackerScore(double timestampNow); // tracker with highest score wins
    float getLocAge(double timestampNow);
    double getLastVisionTimestamp() const;
    double getLastPokeTimestamp() const;
    Position2D getPosition() const;
    float getVisionConfidence() const;
    bool isTimedOut(double timestampNow) const;
    
    // unique ID
    int getId();
    static int requestId();

private:
    // helpers
    void cleanup(double timestampNow);
    float getPositionScore(Position2D const &positionA, Position2D const &positionB) const;
    Position2D mirror(Position2D const &position) const;
    
private:
    // data members
    int _id;
    Position2D _position;
    float _trackerTimeout;
    float _visionConfidence;
    double _creationTimestamp;
    double _lastVisionTimestamp;
    double _lastPokeTimestamp;
    std::deque<double> _recentTimestamps;
    
};

#endif /* LOCALIZATIONTRACKER_HPP_ */

