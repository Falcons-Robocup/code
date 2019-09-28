 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * arbiterGameData.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef ARBITERGAMEDATA_HPP_
#define ARBITERGAMEDATA_HPP_

#include "gameData.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

class ArbiterGameData : public GameData {
public:
    ArbiterGameData();

    void refresh(const GameData&);

    bool ballIsInNegativeGoal() const;
    bool ballIsInPositiveGoal() const;
    bool ballIsAcrossNegativeGoalline() const;
    bool ballIsAcrossPositiveGoalline() const;
    bool ballIsAcrossSideLine() const;
    bool isScrum() const;

    /* the following attributes are deliberately public */
    TeamID teamLastHoldingBall;    // there is always a team that last touched the ball
    RobotID robotLastHoldingBall;  // there is always a robot that last touched the ball

private:
    typedef boost::accumulators::accumulator_set<float, boost::accumulators::stats<boost::accumulators::tag::rolling_mean > > MovingAverageAccumulator;
    MovingAverageAccumulator ballSpeedAccumulator;
    MovingAverageAccumulator ballToRobotDistanceAccumulatorTeamA;
    MovingAverageAccumulator ballToRobotDistanceAccumulatorTeamB;
};

#endif /* ARBITERGAMEDATA_HPP_ */
