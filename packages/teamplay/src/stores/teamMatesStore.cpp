 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * teamMatesStore.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Coen Tempelaars
 */

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include "int/stores/teamMatesStore.hpp"
#include "int/types/intentionSync.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;

teamMatesStore::teamMatesStore()
{
    _team_mates_including_goalie = teamMates();

    intentionStruct intention;
    intention.actionType = actionEnum::INVALID;
    for(size_t i = 0; i <10; i++)
    {
    	intention.robotID = i;
    	_intentions.push_back(intention);
    }

    intentionSync::getInstance().setUpdateFunctionReceivedIntention(
    		boost::bind(&teamMatesStore::setRobotIntention, this, _1));
}

teamMatesStore::~teamMatesStore() { }

void teamMatesStore::stashRobot(const int robotNumber)
{
    if (!_stashedRobot)
    {
        _stashedRobot = _team_mates_including_goalie.getRobotWithNumber(robotNumber);
        _team_mates_including_goalie.remove(robotNumber);
    }
    else
    {
        std::ostringstream msg;
        msg << "stash is not empty";
        TRACE_ERROR(msg.str());
        throw std::runtime_error(msg.str());
    }
}

void teamMatesStore::restoreStashedRobot(const treeEnum& newRole)
{
    if(_stashedRobot)
    {
        _stashedRobot->setRole(newRole);
        _team_mates_including_goalie.add(*_stashedRobot);
        _stashedRobot = boost::none;
    }
    else
    {
        std::ostringstream msg;
        msg << "stash is empty";
        TRACE_ERROR(msg.str());
        throw std::runtime_error(msg.str());
    }
}

void teamMatesStore::setRobotIntention(intentionStruct intention)
{
	try
	{
		_intentions.at(intention.robotID) = intention;
	}
	catch(std::exception &e)
	{
		std::ostringstream msg;
        msg << "Caught exception: " << e.what();
        TRACE_ERROR(msg.str());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

intentionStruct teamMatesStore::getRobotIntention(uint8_t robotID) const
{
	try
	{
		return _intentions.at(robotID);
	}
	catch(std::exception &e)
	{
		std::ostringstream msg;
		msg << "Caught exception: " << e.what();
		TRACE_ERROR(msg.str());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
