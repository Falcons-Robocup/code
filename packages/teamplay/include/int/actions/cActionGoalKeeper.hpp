 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGoalKeeper.hpp
 *
 *  Created on: May 4, 2016
 *      Author: Tim Kouters
 */

#ifndef CACTIONGOALKEEPER_HPP_
#define CACTIONGOALKEEPER_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionGoalKeeper : public cAbstractAction
{
    public:
        cActionGoalKeeper();
        ~cActionGoalKeeper();

        behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

    private:
        const float _Y_MIN_OFFSET_KEEPER = 0.2;
        const float _Y_MAX_OFFSET_KEEPER = 0.4;
        const float _GOALPOST_OFFSET_KEEPER = 0.25;
        const float _INTERSECT_X_EXTENSION = 1.0;
        const float _OPPONENT_AIM_BALL_SPEED_THRESHOLD = 0.3;
        const float _OPPONENT_AIM_PENALTY_PROXIMITY = 0.4;
        const float _OPPONENT_AIM_RAY_DISTANCE = 4.0;
        const int _DISCRETIZE_NUM_SEGMENTS = 5; // number of possible positions to be on is one larger
        const float _SMOOTH_TIMEFILTER = 0.5; // do not change target faster than this amount of seconds
        float _interceptMinimumSpeed = 2.0;
        float _interceptCaptureRadius = 1.3;
        
        Point2D _goalCenter;
        Point2D _goalPostRight;
        Point2D _goalPostLeft;
        Vector2D _leftPosGoalLine;
        Vector2D _rightPosGoalLine;
        Vector2D _penaltyPos;
        
        void getSettings(); // fill in POI's etc
        void parseParameters(const std::map<std::string, std::string> &parameters);
        void clipTarget(Position2D &targetPos);
        void discretize(Position2D &targetPos);
        void timeFilter(Position2D &targetPos);
        bool respondToOpponentAim(Position2D &opponentPos);
        Vector2D opponentAimTarget(Position2D const &opponentPos);
};

#endif /* CACTIONGOALKEEPER_HPP_ */
