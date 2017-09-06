 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mainROS.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: Tim Kouters
 */

#include <ros/ros.h>

#include "ext/worldModelSyncNames.hpp"
#include "int/adapters/configuratorWorldModelPacketROS.hpp"
#include "int/adapters/configuratorMixedTeamPacketROS.hpp"
#include "int/adapters/wmInfoPacketROS.hpp"
#include "int/adapters/wmInfoUDPPacketROS.hpp"
#include "int/adapters/mixedTeamInfoPacketROS.hpp"
#include "int/transceivers/receiverWorldModel.hpp"
#include "int/transceivers/transmitterWorldModel.hpp"
#include "int/transceivers/receiverMixedTeam.hpp"
#include "int/transceivers/transmitterMixedTeam.hpp"

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
		transmitterWorldModel trnsWorldModel;
		receiverMixedTeam rcvrMixedTeam;
		transmitterMixedTeam trnsMixedTeam;

		/* Create configurators */
		configuratorWorldModelPacketROS cnfWorldModelROS;
		configuratorMixedTeamPacketROS cnfMixedTeamROS;

		/* Create adapters */
		wmInfoPacketROS wmPacketROS;
		wmInfoUDPPacketROS wmUDPPacketROS;
		mixedTeamInfoPacketROS mtPacketROS;
		/*
		 * TODO: Add receiver as well, currently we only transmit
		 */

		/* Load default parameters before initing ROS */
		TRACE("Load default parameters");
		cnfWorldModelROS.loadDefaultParameters();
		cnfMixedTeamROS.loadDefaultParameters();

		/* Connect function calls */
		cnfWorldModelROS.addNotifyNewConfigFunction(boost::bind(&receiverWorldModel::reconnect, &rcvrWorldModel));
		cnfWorldModelROS.addNotifyNewConfigFunction(boost::bind(&transmitterWorldModel::reconnect, &trnsWorldModel));
		cnfMixedTeamROS.addNotifyNewConfigFunction(boost::bind(&receiverMixedTeam::reconnect, &rcvrMixedTeam));
		cnfMixedTeamROS.addNotifyNewConfigFunction(boost::bind(&transmitterMixedTeam::reconnect, &trnsMixedTeam));

		wmPacketROS.setUpdateFunction(boost::bind(&transmitterWorldModel::sendPacket, &trnsWorldModel, _1));
		rcvrWorldModel.setNotifyNewPacket(boost::bind(&wmInfoUDPPacketROS::notifyNewUDPPacket, &wmUDPPacketROS, _1));
		mtPacketROS.setUpdateFunction(boost::bind(&transmitterMixedTeam::sendPacket, &trnsMixedTeam, _1));

		TRACE("Connected function calls");
		
		/* Initialize ROS part of adapters */
		cnfWorldModelROS.initializeROS();
		cnfMixedTeamROS.initializeROS();
		wmPacketROS.initializeROS();
		wmUDPPacketROS.initializeROS();
		mtPacketROS.initializeROS();
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
