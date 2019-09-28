 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mainRobotControl.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Jan Feitsma
 */


// system includes
#include <iostream>
#include <string>
#include <iterator>
#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// internal includes
#include "int/adapters/cInputUdpAdapter.hpp"

// other packages
#include "FalconsCommon.h"
#include "tracing.hpp"
#include "ports.hpp"     // from facilities/networkUDP
#include "addresses.hpp" // from facilities/networkUDP

// local defines and namespaces
#define DEFAULT_BURST_WINDOW    1.0
namespace po = boost::program_options;
using namespace std;

void reconnectThread(cInputUdpAdapter * objectAdapter)
{
    connectionType connection = GetPrimaryConnectionType();

    while(1)
    {
        connectionType tmpConnection = GetPrimaryConnectionType(); 
        if(connection != tmpConnection)
        {
            connection = tmpConnection;
            TRACE("reconnecting");
            objectAdapter->reconnect();
        }
        boost::this_thread::sleep_for(boost::chrono::seconds{5});
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotControl", ros::init_options::AnonymousName);
    connectionType currentConnectionType = connectionType::INVALID;
    
    /*
     * Setup option parser
     */
    po::options_description desc("robotControl options");
    string multicastAddress = Facilities::Network::getMulticastAddress();
    int multicastPort = Facilities::Network::getPort(Facilities::Network::PORT_ROBOTCONTROL, getRobotNumber());
    float burstWindow = DEFAULT_BURST_WINDOW;
    desc.add_options()
        ("help,h", "produce help message")
        ("address,a", po::value<string>(&multicastAddress)->default_value(multicastAddress), "IP address to listen to (UDP)")
        ("port,p", po::value<int>(&multicastPort)->default_value(multicastPort), "port to listen to (UDP)")
        ("burst,b", po::value<float>(&burstWindow)->default_value(DEFAULT_BURST_WINDOW), "repeated commands (minibursts) within this time interval are filtered")
    ;
    po::variables_map vm;        
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    
    if (vm.count("help")) {
        cout << desc << "\n";
        return 0;
    }
    
    /*
     * Workaround for slow WIFI boot: wait until we have a 172.* or 192.* or 10.* IP address
     */
    while (1)
    {
        // check
    	currentConnectionType = GetPrimaryConnectionType();

    	if((currentConnectionType == connectionType::WAN) ||
    	   (currentConnectionType == connectionType::LAN) ||
    	   (currentConnectionType == connectionType::USB))
    	{
        	break;
    	}
        // wait
    	TRACE("waiting until network is online ...");
    	sleep(3);
    }
    TRACE("network is online");

    /*
     * Create listener object, which will control the cRobotControl instance (event based)
     */
    cInputUdpAdapter obj(multicastAddress, multicastPort, burstWindow);

    /*
     * Main loop
     */
    ros::NodeHandle _n;

    /*
     * Create thread for reconnect check without block other stuff
     * Verify every 5 seconds whether a reconnect is needed
     */
    boost::thread reconnectCheck(reconnectThread, &obj);

    ros::spin();
}

