// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotStore.hpp
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

#include "int/types/FieldDimensions.hpp"
#include "int/types/Robot.hpp"

namespace teamplay
{

class RobotStore {
public:
    static RobotStore& getInstance()
    {
        static RobotStore instance;
        return instance;
    }

    virtual void clear();
    virtual void addOwnRobot (const Robot&);
    virtual void addTeammate (const Robot&);

    virtual void exchangeOwnRobotWith (const int);
    virtual void undoExchange();

    virtual const Robot& getOwnRobot() const;
    virtual void setOwnRobotRole (const RoleEnum&);

    virtual std::vector<Robot> getAllRobots() const;
    virtual std::vector<Robot> getAllRobotsExclGoalie() const;
    virtual std::vector<Robot> getAllRobotsExclOwnRobot() const;

    virtual std::vector<Robot> getAllRobotsSortedByDistanceTo(const Point2D&) const;
    virtual std::vector<Robot> getAllRobotsExclGoalieSortedByDistanceTo(const Point2D&) const;
    virtual std::vector<Robot> getAllRobotsExclLowestIDSortedByDistanceTo(const Point2D&) const;

    virtual std::vector<Robot> getAllRobotsInArea(const FieldArea&) const;
    virtual std::vector<Robot> getAllRobotsExclGoalieInArea(const FieldArea&) const;

    virtual std::vector<Robot> getAllRobotsExclOwnRobotInArea(const FieldArea&) const;
    virtual std::vector<Robot> getAllRobotsExclOwnRobotExclGoalieInArea(const FieldArea&) const;

    virtual boost::optional<Robot> getAssistantOfOwnRobot() const;
    virtual boost::optional<Robot> getAssistantOfRole (const RoleEnum&) const;
    virtual boost::optional<Robot> getRobotWithRole (const RoleEnum&) const;

private:
    RobotStore();
    virtual ~RobotStore();
    RobotStore(RobotStore const&); // Don't implement
    void operator= (RobotStore const&); // Don't implement

    std::vector<Robot> _all_active_robots;
    boost::optional<int> _number_of_previously_own_robot;
};


} /* namespace teamplay */

#endif /* ROBOTSTORE_HPP_ */
