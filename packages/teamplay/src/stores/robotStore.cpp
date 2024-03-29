// Copyright 2017-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotStore.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace teamplay;

/* Functions with file scope */
static std::vector<robot>::iterator findOwnRobot(std::vector<robot>& robots)
{
    auto retVal = std::find_if(robots.begin(), robots.end(),
            [&](const robot& it){ return it.isOwnRobot(); });

    if (retVal == robots.end())
    {
        std::ostringstream msg;
        msg << "Error: own robot not found";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    return retVal;
}

static std::vector<robot>::iterator findRobotWithNumber(std::vector<robot>& robots, const int number)
{
    auto retVal = std::find_if(robots.begin(), robots.end(),
            [&](const robot& it){ return number == it.getNumber(); });

    if (retVal == robots.end())
    {
        std::ostringstream msg;
        msg << "Error: cannot find robot with number: " << std::to_string(number);
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }

    return retVal;
}


/* Implementation of class robotStore */
robotStore::robotStore() { }

robotStore::~robotStore() { }

void robotStore::clear()
{
    _all_active_robots.clear();
}

void robotStore::addOwnRobot(const robot& r)
{
    auto own_robot = r;
    own_robot.setOwnRobot();
    _all_active_robots.push_back(own_robot);
}

void robotStore::addTeammate(const robot& r)
{
    auto teammate = r;
    teammate.setNotOwnRobot();
    _all_active_robots.push_back(teammate);
}

void robotStore::exchangeOwnRobotWith(const int teammate_number)
{
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

void robotStore::undoExchange()
{
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

robot robotStore::getOwnRobot() const
{
    auto own_robot = std::find_if(_all_active_robots.begin(), _all_active_robots.end(),
            [&](const robot& it){ return it.isOwnRobot(); });

    if (own_robot == _all_active_robots.end())
    {
        std::ostringstream msg;
        msg << "Error: own robot not found";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }
    else
    {
        // Disabled due to (useless) TRACE pollution.
        // Please re-enable if deemed useful.
        //TRACE("getOwnRobot: ") << own_robot->str();
    }

    return *own_robot;
}

void robotStore::setOwnRobotRole (const treeEnum& r)
{
    if (!role(r).isSafeOnMultipleRobots())
    {
        if (std::any_of(_all_active_robots.begin(), _all_active_robots.end(),
                [&](const robot& it){ return ((r == it.getRole()) && (!it.isOwnRobot())); }))
        {
            TRACE_WARNING("Multiple robots have role: %s", role(r).str().c_str());
        }
    }

    auto own_robot = findOwnRobot(_all_active_robots);
    own_robot->setRole(r);
}

/* Getters */
std::vector<robot> robotStore::getAllRobots() const
{
    return _all_active_robots;
}

std::vector<robot> robotStore::getAllRobotsExclGoalie() const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return (it.getRole() != treeEnum::R_GOALKEEPER); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsExclGoalie: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclOwnRobot() const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return !it.isOwnRobot(); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsExclOwnRobot: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsSortedByDistanceTo(const Point2D& p) const
{
    std::vector<robot> retVal = _all_active_robots;

    std::sort(retVal.begin(), retVal.end(),
            [&](const robot& first, const robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const robot& it){ TRACE("getAllRobotsSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclGoalieSortedByDistanceTo (const Point2D& p) const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return (it.getRole() != treeEnum::R_GOALKEEPER); });

    std::sort(retVal.begin(), retVal.end(),
            [&](const robot& first, const robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const robot& it){ TRACE("getAllRobotsExclGoalieSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclLowestIDSortedByDistanceTo (const Point2D& p) const
{
    std::vector<robot> retVal = _all_active_robots;

    std::sort(retVal.begin(), retVal.end(),
            [&](const robot& first, const robot& second)
            { return first.getNumber() > second.getNumber(); });

    retVal.pop_back();

    std::sort(retVal.begin(), retVal.end(),
            [&](const robot& first, const robot& second)
            { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    std::for_each(retVal.begin(), retVal.end(),
            [&](const robot& it){ TRACE("getAllRobotsExclLowestIDSortedByDistanceTo: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsInArea(const fieldArea& area) const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return it.isInArea(area); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsInArea: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclGoalieInArea (const fieldArea& area) const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return (it.getRole() != treeEnum::R_GOALKEEPER) && it.isInArea(area); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsExclGoalieInArea: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclOwnRobotInArea(const fieldArea& area) const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return !it.isOwnRobot() && it.isInArea(area); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsExclOwnRobotInArea: ") << it.str(); });
    return retVal;
}

std::vector<robot> robotStore::getAllRobotsExclOwnRobotExclGoalieInArea(const fieldArea& area) const
{
    std::vector<robot> retVal;

    std::copy_if(_all_active_robots.begin(), _all_active_robots.end(), std::back_inserter(retVal),
            [&](const robot& it){ return !it.isOwnRobot() && (it.getRole() != treeEnum::R_GOALKEEPER) && it.isInArea(area); });

    // Disabled due to (useless) TRACE pollution.
    // Please re-enable if deemed useful.
    //std::for_each(retVal.begin(), retVal.end(),
    //        [&](const robot& it){ TRACE("getAllRobotsExclOwnRobotExclGoalieInArea: ") << it.str(); });
    return retVal;
}

boost::optional<robot> robotStore::getAssistantOfOwnRobot() const
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

boost::optional<robot> robotStore::getAssistantOfRole (const treeEnum& r) const
{
    auto assistant_role = role(r).getAssistantRole();
    if (assistant_role)
    {
        return getRobotWithRole(*assistant_role);
    }
    else //No assistant defined
    {
        TRACE("getAssistantOfRole ") << role(r).str() << ": not found";
        return boost::none;
    }
}

boost::optional<robot> robotStore::getRobotWithRole (const treeEnum& r) const
{
    auto result = std::find_if(_all_active_robots.begin(), _all_active_robots.end(),
            [&](const robot& it){ return it.getRole() == r; });

    if (result != _all_active_robots.end())  // found
    {
        TRACE("getRobotWithRole ") << role(r).str() << ": " << result->str();
        return *result;
    }
    else  // not found
    {
        TRACE("getRobotWithRole ") << role(r).str() << ": not found";
        return boost::none;
    }
}
