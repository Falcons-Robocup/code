 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRobotControlProcessses.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: Jan Feitsma
 */

#include <vector>
#include <exception>

#include "int/cRobotControl.hpp"
#include "int/adapters/cProcessAdapter.hpp"

#include "FalconsCommon.h"
#include "tracing.hpp"
#include <cDiagnostics.hpp>

using namespace std;

void cRobotControl::jobStart(vector<string> args)
{
    TRACE("jobStart");
    try
    {
        cProcessAdapter::getInstance().jobStart(args[0]);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("jobStart failed");
        throw e;
    }
}

void cRobotControl::jobStop(vector<string> args)
{
    TRACE("jobStop");
    try
    {
        cProcessAdapter::getInstance().jobStop(args[0]);
        // clear all persistent service connections
        reset(); 
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("jobStop failed");
        throw e;
    }
}

void cRobotControl::restartSw(vector<string> args)
{
    TRACE("restart software");
    try
    {
        cProcessAdapter::getInstance().restartSw();
        // clear all persistent service connections
        reset(); 
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("reboot failed");
        throw e;
    }
}

void cRobotControl::reset()
{
    // nothing to do anymore, work is now done by robotCLI.py
}

