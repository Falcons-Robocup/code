 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * wmInfoUDPPacketROScoach.cpp
 *
 *  Created on: May 7, 2017
 *      Author: Jan Feitsma
 */

#include "int/adapters/wmInfoUDPPacketROScoach.hpp"

#include "int/types/packetStructureWorldModel.hpp"
#include "int/constructors/packetConstructorWorldModel.hpp"
#include "int/configurators/configuratorWorldModelPacket.hpp"

#include "rosMsgs/t_diag_vision_v2.h"

#include "cDiagnosticsEvents.hpp"
#include "tracer.hpp"
#include "FalconsCommon.h" // getRobotNumber
#include "linalgcv.hpp" // from package geometry, for FCS calculation
#include <ros/ros.h>

wmInfoUDPPacketROScoach::wmInfoUDPPacketROScoach()
{
    TRACE("construct");
}

wmInfoUDPPacketROScoach::~wmInfoUDPPacketROScoach()
{
    TRACE(">");

    TRACE("<");
}

void wmInfoUDPPacketROScoach::initializeROS()
{
    TRACE("initializeROS");
    try
    {
        _hROS.reset(new ros::NodeHandle());

        // create topic publishers
        for (int irobot = 1; irobot <= 6; ++irobot)
        {
            std::string topicname = "/teamA/robot" + boost::lexical_cast<std::string>(irobot) + "/g_diag_vision_v2";
            _publishers[irobot] = _hROS->advertise<rosMsgs::t_diag_vision_v2>(topicname, 100);
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

Vector3D wmInfoUDPPacketROScoach::convertBallMeasurementToFcs(ballMeasurement const &m)
{
    return object2fcs(m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi, m.azimuth, m.elevation, m.radius);
}

Vector3D wmInfoUDPPacketROScoach::convertObstacleMeasurementToFcs(obstacleMeasurement const &m)
{
    return object2fcs(m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi, m.azimuth, m.elevation, m.radius);
}

void wmInfoUDPPacketROScoach::notifyNewUDPPacket(Facilities::Network::cByteArray array)
{
    // coach listener: to visualize, diagnose
    // transform raw measurements into FCS positions which visualizer can draw easily
    // TODO filter duplicates from wmSync buffers, since repeating is a matter internal to wmSync 
    // publish on topic g_diag_vision_v2 because this is in essence vision diagnostics (very similar to v1)
    // TODO consider subsampling? otherwise 30Hz per robot, may be a bit heavy on coach CPU load?
    try
    {
        packetConstructorWorldModel packetConstructor;
        packetConstructor.setByteArray(array);
        packetStructureWorldModel wmStruct = packetConstructor.getWorldModelStructure();

        /*
        * Only sync if part of group ID and not own robotID
        */
        if((wmStruct.groupID == configuratorWorldModelPacket::getInstance().getGroupID()) &&
            (wmStruct.robotID != getRobotNumber()) &&
            (wmStruct.packetVersion = 2) )
        {
            // initialize output message
            rosMsgs::t_diag_vision_v2 msg;
            
            // balls
            for(size_t i = 0; ((i < NR_BALL_MEASUREMENTS) && (wmStruct.balls[i].isValid)); i++)
            {
                rosMsgs::t_object_measurement ball;
                Vector3D posFcs = convertBallMeasurementToFcs(wmStruct.balls[i]);
                ball.x = posFcs.x;
                ball.y = posFcs.y;
                ball.z = posFcs.z;
                ball.objectID = wmStruct.balls[i].objectID;
                ball.timestamp = wmStruct.balls[i].timestamp;
				rosMsgs::s_camera_type cameraType;
				cameraType.camera_type = wmStruct.balls[i].cameraType;
                ball.cameraType = cameraType;
                msg.balls.push_back(ball);
                // tracing only during development, may be too much for production (CPU load @coach)
                TRACE("ball id=%d  pos=(%6.2f,%6.2f,%6.2f) age=%6.2fs", 
                      ball.objectID, ball.x, ball.y, ball.z, getTimeNow()-ball.timestamp); 
            }

            // obstacles
            for(size_t i = 0; ((i < NR_OBSTACLE_MEASUREMENTS) && (wmStruct.obstacles[i].isValid)); i++)
            {
                // very similar to ball
                rosMsgs::t_object_measurement obstacle;
                Vector3D posFcs = convertObstacleMeasurementToFcs(wmStruct.obstacles[i]);
                obstacle.x = posFcs.x;
                obstacle.y = posFcs.y;
                obstacle.z = posFcs.z;
                obstacle.objectID = wmStruct.obstacles[i].objectID;
                obstacle.timestamp = wmStruct.obstacles[i].timestamp;
				rosMsgs::s_camera_type cameraType;
				cameraType.camera_type = wmStruct.obstacles[i].cameraType;
                obstacle.cameraType = cameraType;
                msg.obstacles.push_back(obstacle);
                // tracing only during development, may be too much for production (CPU load @coach)
                TRACE("obstacle id=%d  pos=(%6.2f,%6.2f,%6.2f) age=%6.2fs", 
                      obstacle.objectID, obstacle.x, obstacle.y, obstacle.z, getTimeNow()-obstacle.timestamp); 
            }

            // publish
            _publishers[(int)wmStruct.robotID].publish(msg);
            TRACE("published %d balls and %d obstacles for robot%d", (int)msg.balls.size(), (int)msg.obstacles.size(), (int)wmStruct.robotID);
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

