 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * teamMates.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Coen Tempelaars
 */

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include "int/types/teamMates.hpp"

#include "int/stores/fieldDimensionsStore.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


teamMates::teamMates() { }

teamMates::~teamMates() { }

/* Robots modifiers */
void teamMates::add(const robot& r)
{
    the_robots.push_back(r);
}

void teamMates::clear()
{
    the_robots.clear();
}

void teamMates::remove(const int i)
{
    robots::iterator it;
    for (it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if (it->getNumber() == i) break;
    }
    the_robots.erase(it);
}

/* Robots queries */
boost::optional<robot> teamMates::getRobotWithNumber(const int robotNumber) const
{
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if (it->getNumber() == robotNumber)
        {
            return *it;
        }
    }
    return boost::none;
}

boost::optional<robot> teamMates::getRobotWithRole(const treeEnum&) const
{
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if (it->getRole() == treeEnum::R_GOALKEEPER)
        {
            return *it;
        }
    }
    return boost::none;
}

robots teamMates::getAllRobots() const
{
	return the_robots;
}


int teamMates::getNumberOfRobots() const
{
    return the_robots.size();
}

int teamMates::getNumberOfRobotsWithIDLowerThan(const int reference_id)
{
    int count = 0;
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if(it->getNumber() < reference_id)
        {
            count++;
        }
    }
    return count;
}

int teamMates::getNumberOfRobotsWithIDHigherThan(const int reference_id)
{
    int count = 0;
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if(it->getNumber() > reference_id)
        {
            count++;
        }
    }
    return count;
}

int teamMates::getNumberOfRobotsInclGoalieInArea(const fieldArea& area) const
{
    int count = 0;
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        auto position = it->getPosition();
        if(fieldDimensionsStore::getFieldDimensions().isPositionInArea(position, area))
        {
            count++;
        }
    }
    return count;
}

int teamMates::getNumberOfRobotsExclGoalieInArea(const fieldArea& area) const
{
    int count = 0;
    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        auto position = it->getPosition();
        auto role = it->getRole();
        if((role != treeEnum::R_GOALKEEPER) && (fieldDimensionsStore::getFieldDimensions().isPositionInArea(position, area)))
        {
            count++;
        }
    }
    return count;
}

boost::optional<robot> teamMates::getAssistantOfRole(const treeEnum& role) const
{
    treeEnum role_to_be_found = treeEnum::INVALID;

    boost::optional<robot> returnVal = boost::none;

    switch (role)
    {
    case treeEnum::ATTACKER_MAIN:
        role_to_be_found = treeEnum::ATTACKER_ASSIST;
        break;

    case treeEnum::DEFENDER_MAIN:
        role_to_be_found = treeEnum::DEFENDER_ASSIST;
        break;

    case treeEnum::ATTACKER_ASSIST:
        role_to_be_found = treeEnum::ATTACKER_MAIN;
        break;

    case treeEnum::DEFENDER_ASSIST:
        role_to_be_found = treeEnum::DEFENDER_MAIN;
        break;
    }

    for (auto it = the_robots.begin(); it != the_robots.end(); it++)
    {
        if (it->getRole() == role_to_be_found)
        {
            returnVal = *it;
        }
    }

	return returnVal;
}
