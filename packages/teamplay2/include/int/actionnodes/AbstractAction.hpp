// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractAction.hpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#ifndef ABSTRACTACTION_HPP_
#define ABSTRACTACTION_HPP_

//#include <stdint.h>
//#include <vector>
//#include <string>
//#include <map>
//#include <boost/assign/list_of.hpp>
//#include <boost/assign/ptr_map_inserter.hpp>
//#include <boost/optional.hpp>
//#include <boost/ptr_container/ptr_map.hpp>
//#include <boost/lexical_cast.hpp>

//#include "int/types/cDecisionTreeTypes.hpp"

//#include "int/cMotionPlanningInterface.hpp"

//#include "area2D.hpp"
//#include "vector2d.hpp"
#include "polygon2D.hpp"
//#include "linepoint2D.hpp"

#include "FalconsRTDB.hpp"
#include "tracing.hpp"

#include "int/types/BehaviorTreeTypes.hpp"

class AbstractAction
{
    public:
        AbstractAction();
        virtual ~AbstractAction();

    protected:
        void moveTo(const Point2D& target, const motionTypeEnum& motionType = motionTypeEnum::NORMAL);
        void pass(const Point2D& target);
        void shoot(const Point2D& target);
        void lobShot (const Point2D& target);
        void getBall(const motionTypeEnum& motionProfile = motionTypeEnum::NORMAL);
        void turnAwayFromOpponent(const Point2D& opponentPos);
        void keeperMove();
        void interceptBall();
        void stop();

        bool positionReached(double x, double y, double xy_threshold);

        behTreeReturnEnum getActionResult();

    private:
        T_ACTION makeAction();

        virtual behTreeReturnEnum execute(bool onStarted) = 0;

        void setForbiddenAreas();
        std::vector<polygon2D> getForbiddenAreas() const;
        std::vector<polygon2D> getForbiddenActionAreas() const;

        behTreeReturnEnum translateActionResultToTreeEnum(T_ACTION_RESULT actionResult) const;
        void executeAction(T_ACTION& actionData) const;

        void moveToCenterIfCurrentPositionInvalid(T_ACTION& actionData) const;
        bool isAnyRobotGettingBall() const;

        void sendIntention(const T_ACTION& actionData) const;


};

#endif /* ABSTRACTACTION_HPP_ */
