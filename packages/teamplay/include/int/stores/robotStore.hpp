// Copyright 2017-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotStore.hpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef ROBOTSTORE_HPP_
#define ROBOTSTORE_HPP_

#include <map>
#include <vector>
#include "boost/optional.hpp"

#include "vector2d.hpp"

#include "int/types/fieldDimensions.hpp"
#include "int/types/robot.hpp"

namespace teamplay
{

class robotStore {
public:
    static robotStore& getInstance()
    {
        static robotStore instance;
        return instance;
    }

    virtual void clear();
    virtual void addOwnRobot (const robot&);
    virtual void addTeammate (const robot&);

    virtual void exchangeOwnRobotWith (const int);
    virtual void undoExchange();

    virtual robot getOwnRobot() const;
    virtual void setOwnRobotRole (const treeEnum&);

    virtual std::vector<robot> getAllRobots() const;
    virtual std::vector<robot> getAllRobotsExclGoalie() const;
    virtual std::vector<robot> getAllRobotsExclOwnRobot() const;

    virtual std::vector<robot> getAllRobotsSortedByDistanceTo(const Point2D&) const;
    virtual std::vector<robot> getAllRobotsExclGoalieSortedByDistanceTo(const Point2D&) const;
    virtual std::vector<robot> getAllRobotsExclLowestIDSortedByDistanceTo(const Point2D&) const;

    virtual std::vector<robot> getAllRobotsInArea(const fieldArea&) const;
    virtual std::vector<robot> getAllRobotsExclGoalieInArea(const fieldArea&) const;

    virtual std::vector<robot> getAllRobotsExclOwnRobotInArea(const fieldArea&) const;
    virtual std::vector<robot> getAllRobotsExclOwnRobotExclGoalieInArea(const fieldArea&) const;

    virtual boost::optional<robot> getAssistantOfOwnRobot() const;
    virtual boost::optional<robot> getAssistantOfRole (const treeEnum&) const;
    virtual boost::optional<robot> getRobotWithRole (const treeEnum&) const;

private:
    robotStore();
    virtual ~robotStore();
    robotStore(robotStore const&); // Don't implement
    void operator= (robotStore const&); // Don't implement

    std::vector<robot> _all_active_robots;
    boost::optional<int> _number_of_previously_own_robot;
};


} /* namespace teamplay */

#endif /* ROBOTSTORE_HPP_ */
