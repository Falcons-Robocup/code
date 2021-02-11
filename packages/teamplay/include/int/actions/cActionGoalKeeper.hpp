// Copyright 2016-2019 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
