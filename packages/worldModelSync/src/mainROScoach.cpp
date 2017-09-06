 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mainROScoach.cpp
 *
 *  Created on: May 7, 2017
 *      Author: Jan Feitsma
 */

#include <ros/ros.h>

#include "ext/worldModelSyncNames.hpp"
#include "int/adapters/configuratorWorldModelPacketROS.hpp"
#include "int/adapters/wmInfoUDPPacketROScoach.hpp"
#include "int/transceivers/receiverWorldModel.hpp"

#include "cDiagnosticsEvents.hpp"
#include "tracer.hpp"

int main(int argc, char **argv)
{
    try
    {
        /* Init ROS */
        TRACE("ros::init");
        ros::init(argc, argv, WorldModelSyncNodeNames::worldModelSyncNodeName);
        TRACE("ros::init succeeded");
        //TRACE_INFO("starting worldModelSync"); // must be after ros::init

        /* Create tranceivers */
        receiverWorldModel rcvrWorldModel;

        /* Create configurators */
        configuratorWorldModelPacketROS cnfWorldModelROS;

        /* Create adapters */
        wmInfoUDPPacketROScoach wmUDPPacketROS;

        /* Load default parameters before initing ROS */
        TRACE("Load default parameters");
        cnfWorldModelROS.loadDefaultParameters();

        /* Connect function calls */
        cnfWorldModelROS.addNotifyNewConfigFunction(boost::bind(&receiverWorldModel::reconnect, &rcvrWorldModel));
        rcvrWorldModel.setNotifyNewPacket(boost::bind(&wmInfoUDPPacketROScoach::notifyNewUDPPacket, &wmUDPPacketROS, _1));

        TRACE("Connected function calls");

        /* Initialize ROS part of adapters */
        cnfWorldModelROS.initializeROS();
        wmUDPPacketROS.initializeROS();
        TRACE("construction complete");

        ros::spin();
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}
