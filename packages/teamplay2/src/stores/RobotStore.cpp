// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotStore.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <type_traits>

#include "int/stores/RobotStore.hpp"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace teamplay;

/* Functions with file scope */
template<typename T, 
    typename Iterator = std::conditional_t<std::is_const_v<T>, typename T::const_iterator, typename T::iterator>
>
static Iterator findOwnRobot(T& robots)
{
    auto retVal = std::find_if(robots.begin(), robots.end(),
            [&](const Robot& it){ return it.isOwnRobot(); });
    if (retVal == robots.end())
    {
        std::ostringstream msg;
        msg << "Error: own Robot not found";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    return retVal;
}

static std::vector<Robot>::iterator findRobotWithNumber(std::vector<Robot>& robots, const int number)
{
    auto retVal = std::find_if(robots.begin(), robots.end(),
            [&](const Robot& it){ return number == it.getNumber(); });

    if (retVal == robots.end())
    {
        std::ostringstream msg;
        msg << "Error: cannot find Robot with number: " << std::to_string(number);
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    return retVal;
}


/* Implementation of class RobotStore */
RobotStore::RobotStore() { }

RobotStore::~RobotStore() { }

void RobotStore::clear()
{
    _all_active_robots.clear();
}

void RobotStore::addOwnRobot(const Robot& r)
{
    auto own_robot = r;
    own_robot.setOwnRobot();
    TRACE("addOwnRobot: ") << own_robot.str();
    _all_active_robots.push_back(own_robot);
}

void RobotStore::addTeammate(const Robot& r)
{
    auto teammate = r;
    teammate.setNotOwnRobot();
    TRACE("addTeammate: ") << teammate.str();
    _all_active_robots.push_back(teammate);
}

void RobotStore::exchangeOwnRobotWith(const int teammate_number)
{
    TRACE("exchangeOwnRobotWith: ") << std::to_string(teammate_number);
    
    if (_number_of_previously_own_robot)
    {
        std::ostringstream msg;
        msg << "Error: another exchange is still in progress";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    auto own_robot = findOwnRobot(_all_active_robots);
    auto new_own_robot = findRobotWithNumber(_all_active_robots, teammate_number);

    _number_of_previously_own_robot = own_robot->getNumber();
    own_robot->setNotOwnRobot();
    new_own_robot->setOwnRobot();
}

void RobotStore::undoExchange()
{
    TRACE("undoExchange");
    if (!_number_of_previously_own_robot)
    {
        std::ostringstream msg;
        msg << "Error: exchange is not in progress";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    auto own_robot = findOwnRobot(_all_active_robots);
    auto prev_own_robot = findRobotWithNumber(_all_active_robots, *_number_of_previously_own_robot);

    own_robot->setNotOwnRobot();
    prev_own_robot->setOwnRobot();
    _number_of_previously_own_robot = boost::none;
}

const Robot& RobotStore::getOwnRobot() const
{
    std::vector<Robot>::const_iterator own_robot = findOwnRobot(_all_active_robots);
    TRACE("getOwnRobot: ") << own_robot->str();
    return *own_robot;
}

void RobotStore::setOwnRobotRole (const RoleEnum& r)
{
    auto own_robot = findOwnRobot(_all_active_robots);
    own_robot->setRole(r);
}

/* Getters */
std::vector<Robot> RobotStore::getAllRobots() const
{
    return _all_active_robots;
}

std::vector<Robot> RobotStore::getAllRobotsExclGoalie() const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return (it.getRole() != RoleEnum::GOALKEEPER); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclGoalie: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclOwnRobot() const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return !it.isOwnRobot(); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclOwnRobot: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsSortedByDistanceTo(const Point2D& p) const
{
    std::vector<Robot> retVal = _all_active_robots;

    std::sort(retVal.begin(), retVal.end(),
            [&](const Robot& first, const Robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclGoalieSortedByDistanceTo (const Point2D& p) const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return (it.getRole() != RoleEnum::GOALKEEPER); });

    std::sort(retVal.begin(), retVal.end(),
            [&](const Robot& first, const Robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclGoalieSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclLowestIDSortedByDistanceTo (const Point2D& p) const
{
    std::vector<Robot> retVal = _all_active_robots;

    std::sort(retVal.begin(), retVal.end(),
            [&](const Robot& first, const Robot& second)
            { return first.getNumber() > second.getNumber(); });

    retVal.pop_back();

    std::sort(retVal.begin(), retVal.end(),
            [&](const Robot& first, const Robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclLowestIDSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsInArea(const FieldArea& area) const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return it.isInArea(area); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsInArea: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclGoalieInArea (const FieldArea& area) const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return (it.getRole() != RoleEnum::GOALKEEPER) && it.isInArea(area); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclGoalieInArea: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclOwnRobotInArea(const FieldArea& area) const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return !it.isOwnRobot() && it.isInArea(area); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclOwnRobotInArea: ") << it.str(); });
    return retVal;
}

std::vector<Robot> RobotStore::getAllRobotsExclOwnRobotExclGoalieInArea(const FieldArea& area) const
{
    std::vector<Robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const Robot& it){ return !it.isOwnRobot() && (it.getRole() != RoleEnum::GOALKEEPER) && it.isInArea(area); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const Robot& it){ TRACE("getAllRobotsExclOwnRobotExclGoalieInArea: ") << it.str(); });
    return retVal;
}

boost::optional<Robot> RobotStore::getAssistantOfOwnRobot() const
{
    auto own_assistant_role = getOwnRobot().getAssistantRole();

    if (own_assistant_role)
    {
        return getRobotWithRole(*own_assistant_role);
    }
    else
    {
        TRACE("getAssistantOfOwnRobot: not found");
        return boost::none;
    }
}

boost::optional<Robot> RobotStore::getAssistantOfRole (const RoleEnum& r) const
{
    auto assistant_role = Role(r).getAssistantRole();
    if (assistant_role)
    {
        return getRobotWithRole(*assistant_role);
    }
    else //No assistant defined
    {
        TRACE("getAssistantOfRole ") << Role(r).str() << ": not found";
        return boost::none;
    }
}

boost::optional<Robot> RobotStore::getRobotWithRole (const RoleEnum& r) const
{
    auto result = std::find_if(_all_active_robots.begin(), _all_active_robots.end(),
            [&](const Robot& it){ return it.getRole() == r; });

    if (result != _all_active_robots.end()) // found
    {
        TRACE("getRobotWithRole ") << Role(r).str() << ": " << result->str();
        return *result;
    }
    else // not found
    {
        TRACE("getRobotWithRole ") << Role(r).str() << ": not found";
        return boost::none;
    }
}