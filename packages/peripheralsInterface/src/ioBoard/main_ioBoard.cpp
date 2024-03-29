// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main_ioBoard.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: Edwin Schreuder
 */

#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

#include <sys/stat.h>

#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "int/ioBoard/IoBoard.hpp"
#include "int/ioBoard/Kicker.hpp"
#include "int/ioBoard/KeeperFrame.hpp"

#include "int/ioBoard/cRTDBAdapterRobotStatus.hpp"
#include "int/ioBoard/cRTDBInputKickerAdapter.hpp"
#include "int/ioBoard/RTDBInputKeeperFrameAdapter.hpp"



using std::atomic;
using std::chrono::milliseconds;
using std::endl;
using std::exception;
using std::thread;

class PeripheralsInterfaceIoBoard
{
public:
    PeripheralsInterfaceIoBoard(string portName);
    ~PeripheralsInterfaceIoBoard();

private:
    IoBoard ioBoard;
    Kicker kicker;
    KeeperFrame _keeperFrame;

    thread robotStatusUpdateThread;
    thread rtdbKickerThread;
    thread _rtdbKeeperFrameThread;
    atomic<bool> robotStatusUpdateRunning;

    cRTDBAdapterRobotStatus rtdbAdapterRobotStatus;
    cRTDBInputKickerAdapter rtdbAdapterKicker;
    RTDBInputKeeperFrameAdapter _rtdbAdapterKeeperFrame;

    void robotStatusUpdate();

    void initializeIoBoard();

    void processRobotStatus(IoBoard::Status data);
    void processRobotInPlay(bool inPlay);
    void processRobotSoftwareOn(bool softwareOn);

    void startSoftware();
    void stopSoftware();

    bool inPlay;
    bool softwareOn;
};

PeripheralsInterfaceIoBoard::PeripheralsInterfaceIoBoard(string portName) :
        ioBoard(portName),
        kicker(ioBoard),
        _keeperFrame(ioBoard),
        rtdbAdapterKicker(kicker),
        _rtdbAdapterKeeperFrame(_keeperFrame)
{
    TRACE(">");

    inPlay = false;
    softwareOn = false;

    initializeIoBoard();

    // Push Kicker configuration to ioBoard when the connection has been established
    kicker.updateConfiguration();

    rtdbKickerThread = thread(&cRTDBInputKickerAdapter::waitForKickerSetpoint, &rtdbAdapterKicker);
    _rtdbKeeperFrameThread = thread(&RTDBInputKeeperFrameAdapter::loop, &_rtdbAdapterKeeperFrame);

    rtdbAdapterRobotStatus.initialize();
    robotStatusUpdateRunning = true;
    
    //robotStatusUpdateThread = thread(&PeripheralsInterfaceIoBoard::robotStatusUpdate, this);
    // Keep thread alive and blocking
    robotStatusUpdate();

TRACE("<");
}

PeripheralsInterfaceIoBoard::~PeripheralsInterfaceIoBoard()
{
    TRACE(">");

    if (robotStatusUpdateThread.joinable())
    {
        robotStatusUpdateRunning = false;
    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::initializeIoBoard()
{
    TRACE(">");

    bool initialized = false;

    while (!initialized)
    {
        try
        {
            ioBoard.initialize();
            initialized = true;
            TRACE_INFO("Connection with IoBoard established.");
        }
        catch (exception &e)
        {
            TRACE_ERROR_TIMEOUT(30.0, "First connection with IoBoard not yet established, retrying.");
        }
    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::robotStatusUpdate()
{
    TRACE(">");

    bool board_online = true;

    while(robotStatusUpdateRunning)
    {
        try
        {
            // Retrieve latest status from the ioBoard and process
            IoBoard::Status status = ioBoard.getStatus();
            processRobotStatus(status);

            if (!board_online)
            {
                TRACE_INFO("Communication with IO board restored!");
                board_online = true;
            }
        }
        catch(exception &e)
        {
            // When communication to the IoBoard fails, set the robot to out of play.
            // This ensures that robot is set to out of play when the EMO is pressed.
            processRobotInPlay(false);

            if (board_online)
            {
                board_online = false;
                TRACE_ERROR("Communication with IO board seems offline!");
            }
        }

        WRITE_TRACE;

        this_thread::sleep_for(milliseconds(500));
    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::processRobotStatus(IoBoard::Status status)
{
    TRACE(">");

    processRobotInPlay(status.inPlay);
    processRobotSoftwareOn(status.softwareOn);

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::processRobotInPlay(bool newInPlay)
{
    TRACE(">");

    // Just publish the inPlay status every 500 ms, this way wordmodel always has
    // the latest
    //    if (newInPlay != inPlay) {
    inPlay = newInPlay;
    rtdbAdapterRobotStatus.setRobotStatus(inPlay);

    if (inPlay)
    {
        TRACE("Robot now in play.");
    }
    else
    {
        TRACE("Robot now out of play.");
    }
    //    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::processRobotSoftwareOn(bool newSoftwareOn)
{
    TRACE(">");

    if (newSoftwareOn != softwareOn)
    {
        try
        {
            if (newSoftwareOn)
            {
                startSoftware();
                TRACE_INFO("Software On switch triggered, software is starting.");
            }
            else
            {
                stopSoftware();
                TRACE_INFO("Software On switch triggered, software is shutting down.");
            }

            softwareOn = newSoftwareOn;
        }

        catch (exception &e)
        {
            TRACE_ERROR("Failed to stop or start software, retrying in 1 second: %s", e.what());
        }
    }

    TRACE("<");
}

// SW start/stop is handled by helper scripts, see package robotControl

void PeripheralsInterfaceIoBoard::startSoftware()
{
    TRACE(">");

    int result = system("robotSwStart");

    if (result != 0)
    {
        throw(runtime_error("robotSwStart returned " + to_string(result)));
    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::stopSoftware()
{
    TRACE(">");

    int result = system("robotSwStop");

    if (result < 0)
    {
        throw(runtime_error("robotSwStop returned " + to_string(result)));
    }

    TRACE("<");
}

string getSerialPortName()
{
    // Do we want to make this configurable?
    return "/dev/ttyS0";
}

int main(int argc, char **argv)
{
    INIT_TRACE("peripheralsInterfaceIoBoard");

    TRACE(">");

    TRACE("Retrieve the serial port name to instantiate the IO board communication.");
    string portName = getSerialPortName();

    TRACE("Starting peripheralsInterface IO board on port %s.", portName.c_str());
    PeripheralsInterfaceIoBoard peripheralsInterfaceIoBoard(portName);

    TRACE("<");
}
