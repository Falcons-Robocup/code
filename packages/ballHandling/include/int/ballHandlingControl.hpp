// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

