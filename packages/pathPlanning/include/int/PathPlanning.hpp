 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanning.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_HPP_
#define PATHPLANNING_HPP_


#include <vector>

// common Falcons headers
#include "ConfigInterface.hpp"

// PathPlanning interfaces
#include "int/InputInterface.hpp"
#include "int/OutputInterface.hpp"
#include "int/AbstractVelocitySetpointController.hpp"

// data struct
#include "int/PathPlanningData.hpp"

// use a short alias
using CFI = ConfigInterface<ConfigPathPlanning>;


class PathPlanning
{
public:
    PathPlanning(CFI *configInterface = NULL, InputInterface *inputInterface = NULL, OutputInterface *outputInterface = NULL);
    ~PathPlanning();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    actionResultTypeEnum iterate();

    // raw calculation based on inputs, useful for unit testing
    actionResultTypeEnum calculate();

    // TODO: add an interface to provide (part of) the configuration
    // example use case: interceptBall calculation needs robot speed capability (maxVelXY)

    // quick&dirty intercept experiment
    // if this works and if this direction the way to go (alternative to tricky tuning?),
    // then generalize towards a (re)setConfig interface, e.g. setConfig("limits.normal.maxVelRz", 0.1);
    void setRzLimitsOverride(float maxVelRz, float maxAccRz)
    {
        data.overrideRzLimits = true;
        data.overrideRzMaxVel = maxVelRz;
        data.overrideRzMaxAcc = maxAccRz;
    }
    void unsetRzLimitsOverride()
    {
        data.overrideRzLimits = false;
    }

public:
    // having these public is convenient for test suite
    PathPlanningData data;
    void prepare();
    void setOutputs();
    void clearVelocitySetpointController();

private:
    // helper functions
    void getInputs();
    diagPathPlanning makeDiagnostics();
    void setupVelocitySetpointController();
    AbstractVelocitySetpointController *getVelocitySetpointController();
    AbstractVelocitySetpointController *setupAndGetVelocitySetpointController();

    CFI                       *_configInterface;
    InputInterface            *_inputInterface;
    OutputInterface           *_outputInterface;
    AbstractVelocitySetpointController *_velocitySetpointController = NULL;
    VelocitySetpointControllerTypeEnum  _currentVelocitySetpointControllerType;

};

#endif

