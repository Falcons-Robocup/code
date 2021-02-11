// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <exception>
#include <functional>
#include <thread>

#include <pwd.h>

#include "FalconsRtDB2.hpp"
#include "ConfigRTDBAdapter.hpp"
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "int/motors/cRTDBInputAdapter.hpp"
#include "int/motors/cRTDBOutputAdapter.hpp"


#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/motors/Motion.hpp"
#include "int/motors/Ballhandlers.hpp"

#include "int/motors/DeviceManager.hpp"
#include "int/motors/Diagnostics.hpp"

#include "int/motors/MotionBoard.hpp"
#include "int/motors/BallhandlerBoard.hpp"

using namespace std;

class peripheralsInterfaceMain {
    public:
        peripheralsInterfaceMain();
        ~peripheralsInterfaceMain();

    private:
        void loadDeviceManager();
        void loadDiagnostics();
        void loadController();

        void startController();

        void initConfiguration();
        void updateConfigurationMotors();
        void updateConfigurationBallHandlers();

        PeripheralsInterfaceData _piData;
        DeviceManager _deviceManager;

        MotionBoard _rightMotionBoard;
        MotionBoard _leftMotionBoard;
        MotionBoard _rearMotionBoard;

        BallhandlerBoard _rightBallhandlerBoard;
        BallhandlerBoard _leftBallhandlerBoard;

        Motion _motion;
        Ballhandlers _ballhandlers;

        // RTDB adapters
        cRTDBInputAdapter _rtdbInputAdapter;
        cRTDBOutputAdapter _rtdbOutputAdapter;

        // RTDB config adapters
        ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_MOTORS>* _configAdapterMotors;
        ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS>* _configAdapterBallHandlers;

        Diagnostics _diagnostics;

        // Timer threads
        thread _diagnosticsThread;

        bool _ballhandlersAvailable;
};

peripheralsInterfaceMain::~peripheralsInterfaceMain() {
    delete _configAdapterMotors;
    delete _configAdapterBallHandlers;
}

peripheralsInterfaceMain::peripheralsInterfaceMain():
    _deviceManager(_piData),
    _rightMotionBoard(MOTION_BOARD_RIGHT, _deviceManager),
    _leftMotionBoard(MOTION_BOARD_LEFT, _deviceManager),
    _rearMotionBoard(MOTION_BOARD_REAR, _deviceManager),
    _rightBallhandlerBoard(BALLHANDLER_BOARD_RIGHT, _deviceManager),
    _leftBallhandlerBoard(BALLHANDLER_BOARD_LEFT, _deviceManager),
    _motion(_piData, _leftMotionBoard, _rightMotionBoard, _rearMotionBoard),
    _ballhandlers(_piData, _leftBallhandlerBoard, _rightBallhandlerBoard),
    _rtdbInputAdapter(_piData),
    _rtdbOutputAdapter(_piData),
    _diagnostics(_piData, _ballhandlersAvailable),
    _ballhandlersAvailable(!isGoalKeeper())
{
    // Load the YAML configuration.
    initConfiguration();

    _diagnosticsThread = thread(&peripheralsInterfaceMain::loadDiagnostics, this);

    loadDeviceManager();

    // Keeps this thread alive and blocking
    loadController();
}

void peripheralsInterfaceMain::loadDeviceManager() {
    _deviceManager.start();
}

void peripheralsInterfaceMain::loadDiagnostics() {
    // Wait one or so second for the boards to be identified.
    this_thread::sleep_for(chrono::milliseconds(1200));
    _diagnostics.start();
}

void peripheralsInterfaceMain::loadController() {
    startController();
    //_controllerThread = thread(&peripheralsInterfaceMain::startController, this);
}

void peripheralsInterfaceMain::startController() {
    TRACE("Starting controller.");

    while (true) {
        // Calculate the time when the next iteration should be started.
        chrono::system_clock::time_point end_time = chrono::system_clock::now() + chrono::milliseconds(10);

        _rtdbInputAdapter.getMotorVelocitySetpoint();
        _rtdbInputAdapter.getBallHandlersMotorSetpoint();

        _motion.update();
        _ballhandlers.update();

        _rtdbOutputAdapter.setMotorFeedback();
        _rtdbOutputAdapter.setBallHandlersFeedback();

        this_thread::sleep_until(end_time);

        WRITE_TRACE;
    }
}

void peripheralsInterfaceMain::initConfiguration() {

    ///// Motors
    _configAdapterMotors = new ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_MOTORS>(CONFIG_PERIPHERALSINTERFACE_MOTORS);
    std::string configFileMotors = determineConfig("Motors");
    _configAdapterMotors->loadYAML(configFileMotors);
    _configAdapterMotors->setConfigUpdateCallback( std::bind(&peripheralsInterfaceMain::updateConfigurationMotors, this) );

    updateConfigurationMotors();


    ///// BallHandlers
    _configAdapterBallHandlers = new ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS>(CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS);
    std::string configFileBH = determineConfig("BallHandlers");
    _configAdapterBallHandlers->loadYAML(configFileBH);
    _configAdapterBallHandlers->setConfigUpdateCallback( std::bind(&peripheralsInterfaceMain::updateConfigurationBallHandlers, this) );

    updateConfigurationBallHandlers();

}

void peripheralsInterfaceMain::updateConfigurationMotors() {

    T_CONFIG_PERIPHERALSINTERFACE_MOTORS configMotors;
    _configAdapterMotors->get(configMotors);

    MotionBoardSettings boardSettings;
    boardSettings.motorPlotEnabled = configMotors.motorPlotEnabled;
    boardSettings.pid.p = configMotors.Kp;
    boardSettings.pid.i = configMotors.Ki;
    boardSettings.pid.d = configMotors.Kd;
    boardSettings.pid.iTh = configMotors.iTh;
    boardSettings.pid.iMax = configMotors.iMax;
    boardSettings.maxPwmValue = configMotors.PwmMax;
    boardSettings.maxPwmStepValue = configMotors.PwmMaxDeltaSize;

    // Set the settings to the motorcontrollers.
    _piData.getLeftMotionBoard().setSettings(boardSettings);
    _piData.getRightMotionBoard().setSettings(boardSettings);
    _piData.getRearMotionBoard().setSettings(boardSettings);

}

void peripheralsInterfaceMain::updateConfigurationBallHandlers() {

    T_CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS configBH;
    _configAdapterBallHandlers->get(configBH);

    BallhandlerBoardSettings settings;
    settings.ballhandlerPlotEnabled = configBH.ballhandlerPlotEnabled;
    settings.pid.p = configBH.Kp;
    settings.pid.i = configBH.Ki;
    settings.pid.d = configBH.Kd;
    settings.pid.iTh = configBH.iTh;
    settings.pid.iMax = configBH.iMax;
    settings.anglePid.p = configBH.Ang_Kp;
    settings.anglePid.i = configBH.Ang_Ki;
    settings.anglePid.d = configBH.Ang_Kd;
    settings.anglePid.iTh = 0;
    settings.maxPwmValue = configBH.PwmMax;
    settings.maxPwmStepValue = configBH.PwmMaxDeltaSize;

    // Set the settings to the motorcontrollers.
    _piData.getLeftBallhandlerBoard().setSettings(settings);
    _piData.getRightBallhandlerBoard().setSettings(settings);

}

int main(int argc, char **argv) {

    INIT_TRACE;

    // stays alive and blocking by the controller
    peripheralsInterfaceMain peripheralsInterface;

}
