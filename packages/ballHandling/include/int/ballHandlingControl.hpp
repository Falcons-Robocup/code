 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballHandlingControl.hpp
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#ifndef BALLHANDLINGCONTROL_HPP_
#define BALLHANDLINGCONTROL_HPP_

#include <chrono>

#include "falconsCommon.hpp"
#include "ConfigInterface.hpp"

#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "types/ballHandlersStatusType.hpp"
#include "types/ballHandlersSetpointsType.hpp"

class ballHandlingControl
{
public:
    ballHandlingControl(cRTDBOutputAdapter& rtdbOutputAdapter, ConfigInterface<ConfigBallHandling> *cfi);
    ~ballHandlingControl();

    void update_status(ballHandlersStatusType);
    void update_enabled(bool enabled);
    void update_robot_velocity(Velocity2D robotVelocity);
    void updateSetpoint();
    void updateFeedback();

    void setConfig(ConfigBallHandling const &config);
    void checkForNewConfig();

private:
    cRTDBOutputAdapter        *_rtdbOutputAdapter;
    ConfigInterface<ConfigBallHandling> *_configInterface;

    ballHandlersStatusType    _status;
    ballHandlersSetpointsType _setpoints;
    ConfigBallHandling        _config;
    BallHandlingRobotConfig   _calibration;

    Velocity2D _robotVelocity;

    std::chrono::high_resolution_clock::time_point start_time;

    bool _enabled;
    bool _ballPossession;
    double _angleLeftFraction;
    double _angleRightFraction;
    bool _armLeftLifted = false;
    bool _armRightLifted = false;
    bool _needCalibrationCheck = false; // triggered by (re)config

    void calculateAngleFractions();
    double calculateAngleFraction(int angle, int downAngle, int upAngle);
    int calculateAngle(double angleFraction, int downAngle, int upAngle);

    void determineRobotHasBall();
    bool calculateArmLifted(double angleFraction);

    void calculateSetpoints();
    void addExtraPullForceWhenOneArmLifted();
    void addVelocityFeedForward();

    void traceData();
    DiagBallHandling makeDiagnostics();
};

#endif /* BALLHANDLINGCONTROL_HPP_ */

