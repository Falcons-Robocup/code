 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cCommandAdapter.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: Jan Feitsma
 */

#include "int/adapters/cCommandAdapter.hpp"

#include <cstdlib>
#include <boost/thread/thread.hpp>

#include "FalconsCommon.h"

using namespace std;

cCommandAdapter::cCommandAdapter()
{
}

void cCommandAdapter::interrupt()
{
    // interrupt any running worker thread
    // thread interruption does not work due not using boost interruption points, so we simply kill robotCLI.py
    // http://www.boost.org/doc/libs/1_37_0/doc/html/thread/thread_management.html#interruption_points
    TRACE(">");
    std::string command = "kill -9 `pgrep -f \"robotCLI.py -r " + boost::lexical_cast<std::string>(getRobotNumber()) + "\"`";
    TRACE("calling system(%s)", command.c_str());
    int r = system(command.c_str());
    TRACE("< return value = %d", r);
}

void cCommandAdapter::worker(string command)
{
    TRACE("> (command='%s')", command.c_str());
    // prefix with the python script which will parse and execute the command
    command = "robotCLI.py -r " + boost::lexical_cast<std::string>(getRobotNumber()) + " " + command;
    // next is blocking, can be interrupted
    TRACE("calling system(%s)", command.c_str());
    int r = system(command.c_str());
    // TODO: if r is not 0, then generate a warning or error?
    TRACE("< return value = %d", r);
}

void cCommandAdapter::execute(string command)
{
    TRACE("> (command='%s')", command.c_str());
    // interrupt any still-running command 
    interrupt();
    // dispatch command execution to a new thread
    boost::thread executeThread = boost::thread(&cCommandAdapter::worker, this, command);
    // no need to join the thread, return immediately, to allow interruption and other commands
    TRACE("<");
}


