 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cMotionPlanner.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#include "int/cMotionPlanner.hpp"
#include "cDiagnostics.hpp"
#include "FalconsCommon.h"

using namespace std;

cMotionPlanner::cMotionPlanner(cWorldModelInterface *wm, cRTDBOutputAdapter *rtdbOutput)
{
    //setAction(STOP);
    _interfaces.wm = wm;
    _interfaces.rtdbOutput = rtdbOutput;

    _queryInterface.connect(wm);

    TRACE("cMotionPlanner constructed");
    //PTRACE("INITIALIZED");
}

cMotionPlanner::~cMotionPlanner()
{
}

bool cMotionPlanner::noAction()
{
    TRACE("> actionPtr=%p", _action);
    bool result = _action == NULL;
    TRACE("< result=%d", result);
    return result;
}

bool cMotionPlanner::checkActionEqual(std::string name)
{
    TRACE("> actionPtr=%p", _action);
    bool result = false;
    if (_action != NULL)
    {
        result = (typeid(*_action).name() == name);
    }
    TRACE("< result=%d", result);
    return result;
}

void cMotionPlanner::clearAction()
{
    TRACE("> actionPtr=%p", _action);
    if (_action != NULL)
    {
        delete _action;
    }
    TRACE("<");
}

void cMotionPlanner::initiateAction(cAbstractAction *action)
{
    TRACE("> actionPtr=%p", _action);
    _action = action;
    _action->connect(&_interfaces);
    TRACE("<");
}

void cMotionPlanner::setActionParameters(std::vector<std::string> const &params)
{
    TRACE("> actionPtr=%p", _action);
    // set the parameters of current action
    assert(_action != NULL);
    _action->setParameters(params);
    TRACE("<");
}

void cMotionPlanner::setActionId(int id)
{
    _actionId = id;
}

actionResult cMotionPlanner::execute()
{
    TRACE_FUNCTION("");

    TRACE("> actionPtr=%p", _action);
    // update worldModel
    _interfaces.wm->update();
    // execute current action
    assert(_action != NULL);
    actionResult result;
    result.id = _actionId;
    result.result = _action->execute();

    // Write ACTION_RESULT to RTDB
    _interfaces.rtdbOutput->setActionResult(result);

    // clear if done
    if (result.result != actionResultTypeEnum::RUNNING)
    {
        TRACE("cleanup");
        delete _action;
        _action = NULL;
    }
    TRACE("< result=%d", (int)result.result);
    return result;
}

const cQueryInterface& cMotionPlanner::getQI()
{
    return _queryInterface;
}
