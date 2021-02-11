// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractAction.hpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#ifndef CABSTRACTACTION_HPP_
#define CABSTRACTACTION_HPP_

#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/assign/ptr_map_inserter.hpp>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/lexical_cast.hpp>

#include "int/types/cDecisionTreeTypes.hpp"

#include "int/cMotionPlanningInterface.hpp"

#include "area2D.hpp"
#include "vector2d.hpp"
#include "linepoint2D.hpp"

#include "tracing.hpp"
#include "FalconsRtDB2.hpp"

// These vectors contain the valid parameter values for different actions.
// e.g. a Move accepts a parameter "target", with as value "ball".
// The allowed values for the parameter target is listed here.
// poi == environment.poi
// area == environment.area

static std::string emptyValue = "tbd";
static std::string poiValue = "poi";
static std::string areaValue = "area";

// This list of values is part of the "default point of interest" list: defaultPOI
static std::vector<std::string> defaultPOI { "ball", "lastKnownBallLocation", "robot", "closestTeammember", "closestAttacker", "closestAttackerOnOppHalf", "closestAttackerToOppGoal", "closestDefender", "closestDefenderToOppGoal", "closestOpponent", "closestOpponentToOwnGoal", "potentialOppAttacker", poiValue };
// This list of values is part of the "default areas" list: defaultArea
static std::vector<std::string> defaultArea { areaValue };
// This list of values is part of the "default shoot types" list: defaultShootTypes
static std::vector<std::string> defaultShootTypes { "shootTowardsGoal", "lobTowardsGoal" };
// This list of values is part of the "default pass types" list: defaultPassTypes
static std::vector<std::string> defaultPassTypes { "passTowardsNearestTeammember", "passTowardsNearestAttacker", "passTowardsFurthestAttacker", "passTowardsNearestAttackerOnOppHalf", "passTowardsTipInPosition", "passTowardsFurthestDefender" };
// This list of values is part of the "motion profiles" defined by pathplanning.
static std::vector<std::string> defaultMotionProfiles { "normal", "setpiece" };

class cAbstractAction
{
    public:
        cAbstractAction();
        virtual ~cAbstractAction();

        virtual behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

        boost::optional<Position2D> getPos2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param);
        boost::optional<Area2D> getArea2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param);

        // key: parameters (e.g. "target")
        // value: pair( vector<string> allowedValues, bool optional ) (e.g. pair( {"ball", "robot"}, false ))
        std::map< std::string, std::pair< std::vector<std::string>, bool > > _actionParameters;
        
        template <typename T>
        T getParameter(const std::map<std::string, std::string> &parameters, std::string const &key)
        {
            T result;
            auto paramValPair = parameters.find(key);
            if (paramValPair != parameters.end())
            {
                std::string value = paramValPair->second;
                if (value.compare(emptyValue) != 0)
                {
                    result = boost::lexical_cast<T>(value);
                }
                return result;
            }
            throw std::runtime_error(std::string("missing parameter " + key));
        }

        template <typename T>
        T getParameter(const std::map<std::string, std::string> &parameters, std::string const &key, T const &defaultValue)
        {
            T result = defaultValue;
            auto paramValPair = parameters.find(key);
            if (paramValPair != parameters.end())
            {
                std::string value = paramValPair->second;
                if (value.compare(emptyValue) != 0)
                {
                    result = boost::lexical_cast<T>(value);
                }
            }
            return result;
        }

    protected:
        behTreeReturnEnum moveTo(double x, double y, const std::string& motionProfile = "normal");
        behTreeReturnEnum pass(float x, float y);
        behTreeReturnEnum shoot(const Point2D& target);
        behTreeReturnEnum lobShot (const Point2D& target);
        behTreeReturnEnum getBall(const std::string& motionProfile);
        behTreeReturnEnum turnAwayFromOpponent(double x, double y);
        behTreeReturnEnum keeperMove(float x);
        behTreeReturnEnum interceptBall();
        void stop();
        bool positionReached(double x, double y);
        bool positionReached(double x, double y, double xy_threshold);

        bool isCurrentPosValid() const;
        bool isTargetPosInsideSafetyBoundaries (const Position2D&) const;
        std::vector<polygon2D> getForbiddenAreas() const;
        std::vector<polygon2D> getForbiddenActionAreas() const;

        Point2D getPreferredPartOfGoal() const;
        void sendIntention();

        double XYpositionTolerance = 0.1;
        T_INTENTION _intention;

    private:
        T_ACTION makeAction();
        int _actionId = 0;
        void setForbiddenAreas();
        behTreeReturnEnum translateActionResultToTreeEnum(T_ACTION_RESULT actionResult);
        behTreeReturnEnum executeAction(T_ACTION const &actionData);
        motionTypeEnum determineMotionType(const std::string& motionProfile);

};

#endif /* CABSTRACTACTION_HPP_ */
