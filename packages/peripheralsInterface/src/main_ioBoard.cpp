 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include <cDiagnostics.hpp>
#include <FalconsCommon.h>
#include "tracing.hpp"
#include <peripheralsInterface/ioBoardConfig.h>
#include "../include/ext/peripheralsInterfaceNames.hpp"

#include "int/ioBoard/IoBoard.hpp"
#include "int/Kicker.hpp"
#include "int/KeeperFrame.hpp"

#include "int/adapters/cRosAdapterKicker.hpp"
#include "int/adapters/cRTDBAdapterRobotStatus.hpp"
#include "int/adapters/cRTDBInputKickerAdapter.hpp"
#include "int/adapters/RTDBInputKeeperFrameAdapter.hpp"



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

    cRosAdapterKicker rosAdapterKicker;
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
        rosAdapterKicker(kicker),
        rtdbAdapterKicker(kicker),
        _rtdbAdapterKeeperFrame(_keeperFrame)
{
    TRACE(">");

    inPlay = false;
    softwareOn = false;

    initializeIoBoard();

    rosAdapterKicker.initialize();
    rtdbKickerThread = thread(&cRTDBInputKickerAdapter::waitForKickerSetpoint, &rtdbAdapterKicker);
    _rtdbKeeperFrameThread = thread(&RTDBInputKeeperFrameAdapter::loop, &_rtdbAdapterKeeperFrame);

    rtdbAdapterRobotStatus.initialize();
    robotStatusUpdateRunning = true;
    robotStatusUpdateThread = thread(&PeripheralsInterfaceIoBoard::robotStatusUpdate, this);

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

void PeripheralsInterfaceIoBoard::startSoftware()
{
    TRACE(">");

    int result = system("robotControl start");

    if (result != 0)
    {
        throw(runtime_error("robotControl start returned " + to_string(result)));
    }

    TRACE("<");
}

void PeripheralsInterfaceIoBoard::stopSoftware()
{
    TRACE(">");

    // TODO: Investigate JFEI, robotControl stop doesn't work
    int result = system("jobStopAll");

    if (result < 0)
    {
        throw(runtime_error("jobStopAll returned " + to_string(result)));
    }

    TRACE("<");
}

void loadIoBoardSettings()
{
    TRACE(">");

    // Load the specific IO board settings for this robot.
    loadConfig("IoBoard");

    TRACE("<");
}

string getSerialPortName()
{
    TRACE(">");

    // Create the NodeHandle to read the parameters from
    ros::NodeHandle nh = ros::NodeHandle(peripheralsInterfaceNodeNames::ioBoard);

    // Read parameters from NodeHandle
    peripheralsInterface::ioBoardConfig config;
    config.__fromServer__(nh);

    TRACE("<");
    return config.PortName;
}

int main(int argc, char **argv)
{
    TRACE(">");

    TRACE("Starting ROS for name %s.", peripheralsInterfaceNodeNames::ioBoard);
    ros::init(argc, argv, peripheralsInterfaceNodeNames::ioBoard);

    TRACE("Loading IO board settings");
    loadIoBoardSettings();

    TRACE("Retrieve the serial port name to instantiate the IO board communication.");
    string portName = getSerialPortName();

    TRACE("Starting peripheralsInterface IO board on port %s.", portName);
    PeripheralsInterfaceIoBoard peripheralsInterfaceIoBoard(portName);

    TRACE("ROS spin.");
    ros::spin();

    TRACE("<");
}
