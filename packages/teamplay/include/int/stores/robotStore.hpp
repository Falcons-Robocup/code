 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
