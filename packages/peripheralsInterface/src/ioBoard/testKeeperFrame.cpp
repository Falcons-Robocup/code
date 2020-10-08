 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

