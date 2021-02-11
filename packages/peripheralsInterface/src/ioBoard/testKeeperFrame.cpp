// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * test.cpp        
 *                     For testing the keeper frame
 *    Created on: Jan 31, 2019
 *            Author: Sofia 
 */

#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>

#include <sys/stat.h>

#include "cDiagnostics.hpp"
#include "tracing.hpp"
#include "falconsCommon.hpp"

#include "int/ioBoard/IoBoard.hpp"
#include "int/ioBoard/Kicker.hpp"



using std::atomic;
using std::chrono::milliseconds;
using std::endl;
using std::exception;

class PeripheralsInterfaceIoBoard
{
public:
    PeripheralsInterfaceIoBoard(string portName);
    ~PeripheralsInterfaceIoBoard();

    void run();

private:
    IoBoard ioBoard;
    Kicker kicker;

    void initializeIoBoard();


};

PeripheralsInterfaceIoBoard::PeripheralsInterfaceIoBoard(string portName) :
ioBoard(portName),
kicker(ioBoard)
{
    TRACE(">");

    initializeIoBoard();

    TRACE("<");
}

PeripheralsInterfaceIoBoard::~PeripheralsInterfaceIoBoard()
{
    TRACE(">");

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

void PeripheralsInterfaceIoBoard::run()
{
    int keeper_frame_command = 0;

    while(true)
    {
        keeper_frame_command = getchar();
        if( keeper_frame_command == 'R' )
        {
            ioBoard.setKeeperFrameRight();
        }
        else if( keeper_frame_command == 'U' )
        {
            ioBoard.setKeeperFrameUp();
        }
        else if( keeper_frame_command == 'L' )
        {
            ioBoard.setKeeperFrameLeft();
        }
        else
        {
            keeper_frame_command = 0;
        }
    }
}

int main(int argc, char **argv)
{
    TRACE(">");

    PeripheralsInterfaceIoBoard piIoBoard("/dev/ttyS0");

    piIoBoard.run();

    TRACE("<");
}

