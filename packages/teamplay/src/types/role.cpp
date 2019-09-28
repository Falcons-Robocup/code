 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * role.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <string>

#include "int/types/role.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


role::role() : _role(treeEnum::R_ROBOT_STOP) { }

role::role(const treeEnum& r)
{
    setRole(r);
}

role::~role() { }

treeEnum role::getRole() const
{
    return _role;
}

void role::setRole (const treeEnum& r)
{
    if ((r < treeEnum::ATTACKER_MAIN) || (r > treeEnum::R_ROBOT_STOP))
    {
        std::ostringstream msg;
        msg << "Error: attempt to set an invalid role";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }
    else
    {
        _role = r;
    }
}

boost::optional<treeEnum> role::getAssistantRole() const
{
    switch (_role)
    {
    case treeEnum::ATTACKER_MAIN:
        return treeEnum::ATTACKER_ASSIST;

    case treeEnum::ATTACKER_ASSIST:
        return treeEnum::ATTACKER_MAIN;

    case treeEnum::DEFENDER_MAIN:
        return treeEnum::DEFENDER_ASSIST;

    case treeEnum::DEFENDER_ASSIST:
        return treeEnum::DEFENDER_MAIN;

    default:
        return boost::none;
    }
}

bool role::isSafeOnMultipleRobots() const
{
    // Currently, only R_ROBOT_STOP is safe on multiple robots
    return (_role == treeEnum::R_ROBOT_STOP);
}

std::string role::str() const
{
    try
    {
        std::map<std::string, treeEnum>::const_iterator it;
        for(it = treeEnumMapping.begin(); it != treeEnumMapping.end(); ++it)
        {
            if(it->second == _role)
            {
                return it->first;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("Failed to parse treeEnum '" + std::to_string((int)_role) + "' to std::string. Linked to: " + e.what()));
    }
    return "INVALID";
}
