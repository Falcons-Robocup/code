 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRobotHealth.cpp
 *
 *  Created on: Jan 17, 2016 
 *      Author: Jan Feitsma
 * (moved and stripped, original was in package robotControl)
 */

#include <string>
#include <iostream>
#include <exception>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "int/cRobotHealth.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "ext/cDiagnostics.hpp"

using namespace std;

boost::mutex g_mutex_diag;



cRobotHealth::cRobotHealth()
{
    _rtdb = RtDB2Store::getInstance().getRtDB2(getRobotNumber());
    boost::thread_group threads;
    threads.create_thread(boost::bind(&cRobotHealth::runSlow, this));
    // let the threads run forever
    runFast();
}

cRobotHealth::~cRobotHealth()
{
}

void cRobotHealth::runSlow()
{
    // data struct
    T_DIAG_HEALTH_SLOW msg;
    // loop
    while (true)
    {
        // fill msg
        try
        {
            msg.diskUsage = boost::lexical_cast<int>(systemStdout("getDiskUsage"));
        }
        catch (std::exception &e)
        {
            msg.diskUsage = 0;
        }
        // output & send
        {
            boost::mutex::scoped_lock l(g_mutex_diag);
            _rtdb->put(DIAG_HEALTH_SLOW, &msg);
            tprintf("diskUsage=%d\n", msg.diskUsage);
        }
        // workaround for ntp sync and worldModel sensitivity: force ntpsync occasionally
        try
        {
            int r = system("ntpSync");
            if (r != 0)
            {
                throw std::runtime_error("ntpSync failed");
            }
        }
        catch (std::exception &e)
        {
            TRACE_ERROR("ntpSync failed!");
        }
        // sleep
        sleep(60);
    }
}

void cRobotHealth::runFast()
{
    // data struct
    T_DIAG_HEALTH_FAST msg;
    // loop
    double previous_bytes = 0;
    while (true)
    {
        // fill msg
        try
        {
            std::string tx = systemStdout("getNetworkTx");
            double current_bytes = boost::lexical_cast<double>(tx);
            msg.networkLoad = float((current_bytes - previous_bytes) / 1024.0);
            previous_bytes = current_bytes;
        }
        catch (std::exception &e)
        {
            msg.networkLoad = 0.0;
        }
        try
        {
            std::string cpuload = systemStdout("getCpuLoad");
            msg.cpuLoad = boost::lexical_cast<float>(cpuload);
        }
        catch (std::exception &e)
        {
            msg.cpuLoad = 0.0;
        }
        try
        {
            std::string semCount = systemStdout("getSemCount");
            msg.semCount = boost::lexical_cast<int>(semCount);
        }
        catch (std::exception &e)
        {
            msg.semCount = -1;
        }
        // output & send
        {
            boost::mutex::scoped_lock l(g_mutex_diag);
            _rtdb->put(DIAG_HEALTH_FAST, &msg);
            tprintf("semCount=%2d networkLoad=%7.2fKB/s cpuLoad=%5.1f", msg.semCount, msg.networkLoad, msg.cpuLoad);
        }
        // sleep
        sleep(1);
        // write tracing
        WRITE_TRACE;
    }
}



