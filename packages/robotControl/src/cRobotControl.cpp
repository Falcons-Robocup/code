 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRobotControl.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Jan Feitsma
 */

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <vector>
#include <exception>

#include "int/cRobotControl.hpp"
#include "int/adapters/cProcessAdapter.hpp"
#include "int/adapters/cCommandAdapter.hpp"
#include "int/adapters/cConfigAdapter.hpp"
// TODO better separate ROS adapters, like in main -- look at Eriks design pattern?

#include "FalconsCommon.h"
#include "tracing.hpp"
#include <cDiagnostics.hpp>

using namespace std;

cRobotControl::cRobotControl()
{
    // register the command execution functions
    _commandMap["jobStart"] = &(this->jobStart);
    _commandMap["jobStop"] = &(this->jobStop);
    _commandMap["restartSw"] = &(this->restartSw);
    _commandMap["setClaim"] = &(this->setClaim);
    _commandMap["releaseClaim"] = &(this->releaseClaim);
    _commandMap["configSet"] = &(this->configSet);
    _commandMap["configSave"] = &(this->configSave);
}

void cRobotControl::parse(string const &command)
{
    // the first two words are always "from hostname", with e.g. hostname=butter
    // we use this to realize developers not interfere with each other - "robot claiming"
    TRACE_INFO("received command: %s", command.c_str());
    
    // parse command, relay to the proper function
    vector<string> strs;
    boost::split(strs, command, boost::is_any_of("\t "));
    
    // TODO replace TRACE_ERROR with TRACE_WARNING
    
    // check protocol
    if (strs.size() < 3)
    {
        TRACE_ERROR("protocol violation: expect at least words FROM HOSTNAME COMMAND, got '%s'", command.c_str());
        return;
    }
    if (strs[0] != "from")
    {
        TRACE_ERROR("protocol violation: expect at least words FROM HOSTNAME COMMAND, got '%s'", command.c_str());
        return;
    }
    
    // check if sender matches with claim
    string sender = strs[1];
    string command0 = strs[2];
    if (_claimedBy.size() && (sender != _claimedBy))
    {
        TRACE_ERROR("somebody is trying to interfere - ignoring");
        return;
    }
    
    // call applicable command function
    // pop "from hostname command"
    strs.erase(strs.begin());
    strs.erase(strs.begin());
    strs.erase(strs.begin());
    if (_commandMap.count(command0))
    {
        TRACE("nargs=%d", strs.size());
        try
        {
            _commandMap[command0](strs);
        }
        catch (std::exception &e)
        {
            TRACE_ERROR("something went wrong while executing command");
            // do not rethrow!
            return;
        }
    }
    else
    {
        // relay to robotCLI.py
        cCommandAdapter::getInstance().execute(command0 + " " + boost::algorithm::join(strs, " "));
        return;
    }
}

