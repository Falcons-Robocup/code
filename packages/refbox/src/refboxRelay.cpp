 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*********************************************************************************
 *
 *  $Id: refboxRelay.cpp 1833 2015-06-16 17:48:51Z tkov $
 *
 *  TITLE: refboxRelay.cpp 
 *
 * Description:
 *
 * written by: Jan Feitsma
 * Based on work from Stefan Schiffer (dr.stf@web.de) and Jan van Mastbergen
 *
 ********************************************************************************
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *******************************************************************************/
#include <cstdlib>
#include <boost/thread/thread.hpp>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <refbox/refboxRelayConfig.h>

#include "FalconsCommon.h"
#include <cDiagnostics.hpp>
#include <cDiagnosticsEvents.hpp>
#include "int/TCPIP_client.h"
#include "int/cRosAdapterLogging.hpp"
#include "cTransmitterUDP.hpp"
#include "ports.hpp"     // from facilities/networkUDP
#include "addresses.hpp" // from facilities/networkUDP
#include "rosMsgs/t_diag_refbox.h"


// GLOBALS 
CTCPIP_Client   *pTCPIP_Client   = NULL;
Facilities::Network::cTransmitterUDP *pTransmitterUDP = NULL; // note: the thread created by CTCPIP_Client will inherit this pointer and be the only one to use it
std::string IPAddress;
diagnostics::cDiagnosticsSender<rosMsgs::t_diag_refbox> diagSender(diagnostics::DIAG_REFBOX, 0);
Facilities::Network::cByteArray byteArrayLastCommand;
std::string lastCommand;
boost::mutex mtx;


// Default TCP/IP listener port number for RefBox protocol (NOT managed in our networkUDP ports.hpp!)
static int port = 28097;


// TODO reconsider reconfiguration. Centralize? We should try to not be ROS dependent ...
// so I stepped away from coach/cfg, but for now I define similar enums here
enum teamColorEnum
{ 
    CYAN = 0,
    MAGENTA
};
enum accessEnum 
{ 
    FIELD_A,
    FIELD_B,
    FIELD_C,
    FIELD_D,
    FIELD_FALCONS,
    FIELD_LOCALHOST
};


#define DEFAULT_ACCESS_POINT    FIELD_LOCALHOST
#define DEFAULT_TEAM_COLOR      CYAN
#define OTHER_TEAM_COLOR        MAGENTA
static const unsigned int TRANSMIT_REPEAT = 5; // in addition to the actual send, so 0 is just a single packet
static const unsigned int TRANSMIT_USLEEP = 500; // in microseconds



void send()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    if (byteArrayLastCommand.getSize())
    {
        TRACE("sending packet %s", lastCommand.c_str());
        TRACE("pTransmitterUDP=%p", pTransmitterUDP);
	    pTransmitterUDP->sendPacket(&byteArrayLastCommand);
    }
}

// callback which gets a string and broadcasts it over UDP to our team
void refboxCallBack(char const *command)
{
	TRACE("callback triggered with command '%s'", command);
    TRACE("pTransmitterUDP=%p", pTransmitterUDP);
    // note: the thread created by CTCPIP_Client will inherit this pointer and be the only one to use it
	if (!pTransmitterUDP->isSocketOpen())
	{
    	TRACE_ERROR("socket is not open!!!");
	}

	// send to logger/visualizer
	rosMsgs::t_diag_refbox msg;
	msg.refboxCmd = std::string(command).substr(5);
	diagSender.set(msg); // will trigger send

    // setup the UDP packet
	std::vector<uint8_t> data(command, command + strlen(command));
	{
        boost::lock_guard<boost::mutex> guard(mtx);
        lastCommand = command;
        byteArrayLastCommand.setData(data);
    }
    	
	// transmit with a little repeater, to be robust for poor WIFI
    TRACE("sending packet burst");
	send(); // always 1 transmit, loop is optional
	for (unsigned int i = 0; i < TRANSMIT_REPEAT; ++i)
	{
        usleep(TRANSMIT_USLEEP);
        send();
	}
}



const char *TeamColorString(int teamColor)
{
	if (teamColor == CYAN) {
		return "CYAN";
	} else if (teamColor == MAGENTA) {
		return "MAGENTA";
	} else {
		TRACE("Encountered bad team color enum value");
		return NULL;
	}
}

std::string getIPNumber(int access)
{
    std::string ret_val = "";

    switch (access)
    {
        case FIELD_A:
        {
            ret_val = "172.16.1.2";
            break;
        }

        case FIELD_B:
        {
            ret_val = "172.16.2.2";
            break;
        }

        case FIELD_C:
        {
            ret_val = "172.16.3.2";
            break;
        }

        case FIELD_D:
        {
            ret_val = "172.16.4.2";
            break;
        }

        case FIELD_FALCONS:
        {
            ret_val = "172.16.74.10";
            break;
        }

        case FIELD_LOCALHOST:
        {
            ret_val = "localhost";
            break;
        }

        default:
        {
            TRACE("Received invalid access point id: %d", access);
            ret_val = "localhost";
            break;
        }
    }

    return ret_val;
}

void createClient(const char* server, int teamColor)
{
    TRACE("createClient, server=%s, teamcolor=%d", server, teamColor);

	if (pTCPIP_Client != NULL) {
        TRACE("createClient already started");
        return;
	}

    TRACE("createClient creating");
	pTCPIP_Client = new CTCPIP_Client(server, port, refboxCallBack);
	if (not pTCPIP_Client->isConnected()) {
        TRACE("createClient while not connected");
		delete pTCPIP_Client;
		pTCPIP_Client = NULL;
	}
	else
    {
        TRACE("createClient setting color");
	    pTCPIP_Client->setOwnColor(TeamColorString(teamColor));
        TRACE("createClient done!");
    }
} // createClient

#include <dynamic_reconfigure/server.h>
void reconfig_cb(refbox::refboxRelayConfig &config, uint32_t level)
{
    if (getTeamChar() == 'B')
    {
        TRACE("teamB (MAGENTA) is not suppposed to reconfigure");
        // for simulation the refboxGUI will first prompt teamA, which should be CYAN
        // and once a robot of teamB is activated, then refbox will detect and prompt teamB, which should be MAGENTA
        return;
    }
    
    TRACE("reconfiguring");

	std::string IPaddress = getIPNumber(config.AccessPoint);

    if (pTCPIP_Client != NULL)
    {
	    delete pTCPIP_Client;
	    pTCPIP_Client = NULL;
    }
    	
	createClient(IPaddress.c_str(), config.TeamColor);

} // reconfig_cb



/* *** MAIN *** */
int main(int argc, char ** argv)
{
	IPAddress = getIPNumber(DEFAULT_ACCESS_POINT);
    std::string         multicastAddress = Facilities::Network::getMulticastAddress();
    int                 multicastPort = Facilities::Network::getPort(Facilities::Network::PORT_REFBOXRELAY, 0);
    bool                doFeedbackLogging = true;
    
    ros::init(argc, argv, "refboxRelay");
	ros::NodeHandle nh;
	
	// give globalConfig some time to come online
	sleep(10);
	
    // use global configuration parameter
    int teamcolor = DEFAULT_TEAM_COLOR;
    // simulation workaround
    if (getTeamChar() == 'B')
    {
        teamcolor = OTHER_TEAM_COLOR;
        doFeedbackLogging = false;
    }
    TRACE("ip=%s team='%c' teamcolor=%d port=%d", IPAddress.c_str(), getTeamChar(), teamcolor, multicastPort);

    // setup the UDP transmitter
    pTransmitterUDP = new Facilities::Network::cTransmitterUDP(multicastAddress, multicastPort, true, getHops());
    TRACE("pTransmitterUDP=%p", pTransmitterUDP); 
    // note: the thread created by CTCPIP_Client will inherit this pointer and be the only one to use it
        
    // setup the TCP Refbox listener relay
    createClient(IPAddress.c_str(), teamcolor);

    // Create reconfigure server
    dynamic_reconfigure::Server<refbox::refboxRelayConfig> srv;
    dynamic_reconfigure::Server<refbox::refboxRelayConfig>::CallbackType f;
    // Bind the reconfiguration function
    f = boost::bind(&reconfig_cb, _1, _2);
    srv.setCallback(f); // this will immediately fire the callback hence reset the refbox listener
    
    // loop
    ros::Rate looprate(5.0);
    int counter = 0;
    while(ros::ok())
    {
        TRACE("spinOnce");
        ros::spinOnce();
        // try to create client every once in a while
        if ((++counter % 20) == 0)
        {
            TRACE("periodical create client");
            createClient(IPAddress.c_str(), teamcolor);
        }
        
        // in simulation, not for teamB (in match mode we are always teamA)
        if (doFeedbackLogging && (pTCPIP_Client != NULL))
        {
            TRACE("pTCPIP_Client=%p", pTCPIP_Client);
            cRosAdapterLogging::getInstance().update(pTCPIP_Client);
        }
        // periodical repeat (no-burst) as catch-net for poor wifi
        send();
        TRACE("sleep");
        looprate.sleep();
    }

    // done
    delete pTCPIP_Client;
    return 0;
}
