 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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
 */

#include <string>
#include <iostream>
#include <exception>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include "int/cRobotHealth.hpp"
#include "FalconsCommon.h"
#include "rosMsgs/t_diag_health_slow.h"
#include "rosMsgs/t_diag_health_mid.h"
#include "rosMsgs/t_diag_health_fast.h"
#include "cDiagnostics.hpp"
#include "cDiagnosticsEvents.hpp"

using namespace std;

cRobotHealth::cRobotHealth()
{
    TRACE("starting threads");  
    boost::thread_group threads;
    threads.create_thread(boost::bind(&cRobotHealth::runSlow, this));
    threads.create_thread(boost::bind(&cRobotHealth::runMid, this));
    threads.create_thread(boost::bind(&cRobotHealth::runFast, this));
}

void cRobotHealth::runSlow()
{
    // data struct
    rosMsgs::t_diag_health_slow msg;
    // setup UDP diagnostics sender
    diagnostics::cDiagnosticsSender<rosMsgs::t_diag_health_slow> udpSender(diagnostics::DIAG_HEALTH_SLOW, 0);
    // loop
    while (true)
    {
        TRACE("slow update iteration");
        // fill msg
        try
        {
            msg.cpuFreq = systemStdout("getCpuFreq");
        }
        catch (std::exception &e)
        {
            TRACE("getCpuFreq failed!");
            msg.cpuFreq = "unknown";
        }
        try
        {
            msg.diskUsage = boost::lexical_cast<int>(systemStdout("getDiskUsage"));
        }
        catch (std::exception &e)
        {
            TRACE("getDiskUsage failed!");
            msg.diskUsage = 0;
        }
        try
        {
            msg.gitBranch = systemStdout("getGitBranch");
        }
        catch (std::exception &e)
        {
            TRACE("getGitBranch failed!");
            msg.gitBranch = "unknown";
        }
        try
        {
            msg.gitDirty = systemStdout("getGitDirty");
        }
        catch (std::exception &e)
        {
            TRACE("getGitDirty failed!");
            msg.gitDirty = "unknown";
        }
        try
        {
            msg.globalConfig = systemStdout("configShow");
        }
        catch (std::exception &e)
        {
            TRACE("configShow failed!");
            msg.globalConfig = "unknown";
        }
        // send (implied by set)
        udpSender.set(msg);
        // workaround for ntp sync and worldModel sensitivity: force ntpsync occasionally
        try
        {
            // is a script which only runs if on robot (so not in simulator)
            // and syncs clock to claimer (via a tmp file..)
            system("ntpSync");
        }
        catch (std::exception &e)
        {
            TRACE("ntpSync failed!");
        }
        sleep(60);
    }
}

void cRobotHealth::runMid()
{
    // data struct
    rosMsgs::t_diag_health_mid msg;
    // setup UDP diagnostics sender
    diagnostics::cDiagnosticsSender<rosMsgs::t_diag_health_mid> udpSender(diagnostics::DIAG_HEALTH_MID, 0);
    // loop
    while (true)
    {
        // fill msg
        try
        {
            msg.deadProcesses = systemStdout("getDeadProcesses");
        }
        catch (std::exception &e)
        {
            TRACE("getDeadProcesses failed!");
            msg.deadProcesses = "unknown";
        }
        try
        {
            msg.badOutput = systemStdout("getBadOutput");
        }
        catch (std::exception &e)
        {
            TRACE("getBadOutput failed!");
            msg.badOutput = "unknown";
        }
        try
        {
            msg.accessPoint = systemStdout("getAccessPoint");
        }
        catch (std::exception &e)
        {
            TRACE("getAccessPoint failed!");
            msg.accessPoint = "unknown";
        }
        try
        {
            msg.ipAddress = systemStdout("getIpAddress");
        }
        catch (std::exception &e)
        {
            TRACE("getIpAddress failed!");
            msg.ipAddress = "unknown";
        }
        try
        {
            msg.badPeripherals = systemStdout("getBadPeripherals");
            // generate event?
            if (msg.badPeripherals.size() > 0)
            {
                TRACE_ERROR(msg.badPeripherals.c_str());
            }
        }
        catch (std::exception &e)
        {
            TRACE("getBadPeripherals failed!");
            msg.badPeripherals = "unknown";
        }
        // send (implied by set)
        udpSender.set(msg);
        sleep(10);
    }
}

void cRobotHealth::runFast()
{
    // data struct
    rosMsgs::t_diag_health_fast msg;
    // setup UDP diagnostics sender
    diagnostics::cDiagnosticsSender<rosMsgs::t_diag_health_fast> udpSender(diagnostics::DIAG_HEALTH_FAST, 0);
    // loop
    double previous_bytes = 0;
    while (true)
    {
        // fill msg
        try
        {
            std::string tx = systemStdout("getNetworkTx");
            //std::string rx = systemStdout("getNetworkRx");
            //double current_bytes = boost::lexical_cast<double>(tx) + boost::lexical_cast<double>(rx);
            double current_bytes = boost::lexical_cast<double>(tx);
            msg.networkLoad = float((current_bytes - previous_bytes) / 1024.0);
            //TRACE("tx=%s rx=%s prev=%e curr=%e load=%e", tx.c_str(), rx.c_str(), previous_bytes, current_bytes, msg.networkLoad);
            previous_bytes = current_bytes;
        }
        catch (std::exception &e)
        {
            TRACE("network load get/calc failed!");
            msg.networkLoad = 0.0;
        }
        try
        {
            std::string cpuload = systemStdout("getCpuLoad");
            //TRACE("cpuload=%s", cpuload.c_str());
            msg.cpuLoad = boost::lexical_cast<float>(cpuload);
        }
        catch (std::exception &e)
        {
            TRACE("getCpuLoad failed!");
            msg.cpuLoad = 0.0;
        }
        // send (implied by set)
        udpSender.set(msg);
        sleep(1);
    }
}



