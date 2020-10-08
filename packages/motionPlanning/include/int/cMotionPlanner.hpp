 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cMotionPlanner.hpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef CMOTIONPLANNER_HPP_
#define CMOTIONPLANNER_HPP_

#include "falconsCommon.hpp"
#include "PathPlanningClient.hpp"
#include "int/cAllActions.hpp"
#include "int/cQueryInterface.hpp"


class cMotionPlanner
{
public:
    cMotionPlanner(MP_WorldModelInterface *wm, PathPlanningClient *pp, MP_RTDBOutputAdapter *rtdbOutput);
    ~cMotionPlanner();
    
    // set a the action (either continue current action or start a new one)
    template<typename T>
    void setAction(T const &action)
    {
        TRACE("setAction(%s)", typeid(action).name());
        if (noAction())
        {
            TRACE("first action");
            initiateAction((MP_AbstractAction *)(new T()));
        }
        else if (!checkActionEqual(typeid(action).name()))
        {
            TRACE("changing action");
            clearAction();
            initiateAction((MP_AbstractAction *)(new T()));
        }
        // else do nothing: running action remains alive
    }
    
    // set the parameters of current action
    void setActionParameters(std::vector<std::string> const &params);
    
    // execute current action
    actionResult execute();
    
    // interface getters (needed for relaying forbidden areas & suppressing ballhandlers)
    MP_RTDBOutputAdapter *getRTDBOutput();
    const cQueryInterface& getQI();

private:
    // current action and modifiers
    MP_AbstractAction *_action = NULL;
    bool noAction();
    bool checkActionEqual(std::string name);
    void clearAction();
    void initiateAction(MP_AbstractAction * action);
    
    // interfaces
    cInterfaces _interfaces;
    cQueryInterface _queryInterface;
    
};

#endif /* CMOTIONPLANNER_HPP_ */

