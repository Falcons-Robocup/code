 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: jfeitsma
 * Creation: 2014-09-02
 *
 * RCS simulator, team-level feedback to each robot's worldmodel.
 * Currently worldModel offers the following SET services:
 *  - set_own_location      : ? should this be called at simball?
 *  - set_own_location       : called here
 *  - set_obstacle_location  : called here
 *  - set_member_location    : called here
 */

#include <math.h>
#include <sstream>
#include <simulator/Position.h>
#include <simulator/WorldInfo.h>
#include <simulator/simulatorConfig.h>
#include <simulator/BallPossessionSim.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include "ros/subscriber.h"
#include <std_msgs/Time.h>
#include "std_msgs/UInt8.h"
#include "worldModel/overrule_ball_location.h"
#include "worldModel/overrule_obstacle_location.h"
#include "worldModel/t_wmInfo.h"
#include "worldModel/claim_own_ball_possession.h"
#include "worldModel/release_own_ball_possession.h"
#include "worldModel/set_own_camera_ball_possession.h"
#include "WorldModelNames.h"
#include "FalconsCommon.h" // Tracer
#include <cDiagnosticsEvents.hpp>
#include "ext/robotSimNames.h"

#define _VIEW_DISTANCE_ROBOT 99.0
// disabled for now - robots should all perceive the same ball; 
// actually the logic should be that if ANY robot can see the ball, then ALL must be told the ball location
// but we better leave that implementation for next simulator (Gazebo).
#define _MAX_ROBOTS 7

namespace simulator {

// DECLARATION

class SimTeam {
public:
	SimTeam(const ros::NodeHandle& nh, std::string const &teamname);
private:
	void updateCallback(const WorldInfoConstPtr& worldinfo);
	void worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg);

	ros::NodeHandle m_nh;
	std::string m_teamname;
	WorldInfo m_worldinfo;
	ros::Subscriber m_worldinfo_sub;
	ros::Publisher m_BallPossessionSim_pub;
	double m_last_update;
	double m_update_interval;
	ros::ServiceClient _srvc_ball_location[255];
	//ros::ServiceClient _srvc_ball_possession[255];
	ros::Subscriber    _topic_subWorldModel[255];
	ros::ServiceClient _srvc_obstacle_friend_location[255];
	ros::ServiceClient _srvc_obstacle_enemy_location[255];
	ros::ServiceClient _srvc_obstacle_visualizer[255];
	ros::ServiceClient _srvc_claim_ball[255];
	ros::ServiceClient _srvc_release_ball[255];
	ros::ServiceClient _srvc_CameraBallPossession[255];
	bool robotHasBall;
	uint8_t robotHasBallID;

};
// class SimTeam

// IMPLEMENTATIONS

SimTeam::SimTeam(const ros::NodeHandle& nh, const std::string &teamname) :
		m_nh(nh), m_teamname(teamname) {
	TRACEF("SimTeam constructor, teamname=%s", m_teamname.c_str());
	m_worldinfo_sub = m_nh.subscribe("/simulator/worldinfo", 10,
			&SimTeam::updateCallback, this);

	m_BallPossessionSim_pub = m_nh.advertise<simulator::BallPossessionSim>(RobotsimInterface::S_SIM_BALLPOSSESSION, 100);
	m_last_update = 0;
	m_update_interval = 0.09;
	robotHasBall = false;
	robotHasBallID = 0;

	/* Create subscribes for topics of wmInfo */
	for(size_t i = 0; i < _MAX_ROBOTS; i++)
	{
		std::string ownerName = "/" + m_teamname + "/robot" + std::to_string(i);

		_topic_subWorldModel[i] = m_nh.subscribe(ownerName + "/" + WorldModelInterface::t_wmInfo, 1, &SimTeam::worldModel_cb, this);
	}
}

void SimTeam::worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg)
{
	BallPossessionSim ball_pub;
	ball_pub.possessionType = msg->possession.type;
	ball_pub.possessionRobotID = msg->possession.robotID;
	ball_pub.possessionTeam = m_teamname[4];
	m_BallPossessionSim_pub.publish(ball_pub);
	TRACE("BallPossessionSim type=%d team=%c robotID=%d", ball_pub.possessionType, ball_pub.possessionTeam, ball_pub.possessionRobotID);
}

void SimTeam::updateCallback(const WorldInfoConstPtr& worldinfo) {
	// JFEI tracing shows this function gets called much more than 10x per second
	// and I do not know why. Quick workaround: internal timer, only continue if
	// elapsed time is large enough.
	double t = ros::Time::now().toSec();
	TRACE("pre update t=%16.8e", t);
	if (t <= m_last_update + m_update_interval) {
		TRACE("no update");
		return; // too early to run again
	}
	m_last_update = t;

	int robotIdx = 0;
	TRACE("update");
	m_worldinfo = *worldinfo;
	for (int irobot = 0; irobot < (int) m_worldinfo.robots.size(); ++irobot) 
	{
	    // Identify robotid
	    std::string myname = m_worldinfo.robots[irobot].name;
	    robotIdx = myname[5] - '0'; // assume string "robotN"

		if (m_worldinfo.robots[irobot].team == m_teamname)
		{
		    std_msgs::UInt8 robotID;
		    //std::string myname = m_worldinfo.robots[irobot].name;
		    robotID.data = robotIdx; // assume string "robotN"
            bool playing_left_to_right = (m_worldinfo.robots[irobot].team == "teamA");
            std::string ownerName = "/" + m_teamname + "/" + myname;

		    TRACE("irobot=%d robotID=%d, name=%s", irobot, int(robotID.data), myname.c_str());

		    /*
		     * Verify whether service is already created
		     */
		    if(!_srvc_ball_location[robotIdx].exists())
		    {
		        std::string ball_service_name = ownerName + "/"
                        + WorldModelInterface::s_set_overrule_ball_location;
                ros::service::waitForService(ball_service_name);

                _srvc_ball_location[robotIdx] = m_nh.serviceClient<
                        worldModel::overrule_ball_location>(ball_service_name,
                        USE_SERVICE_PERSISTENCY);
                TRACE_ERROR("Created service");
		    }

		    if(!_srvc_claim_ball[robotIdx].exists())
		    {
		    	std::string strClaimBall = ownerName + "/" +
		    			WorldModelInterface::s_claim_own_ball_possession;
		    	ros::service::waitForService(strClaimBall);

		    	_srvc_claim_ball[robotIdx] = m_nh.serviceClient<
		    			worldModel::claim_own_ball_possession>(strClaimBall, USE_SERVICE_PERSISTENCY);
		    	TRACE_ERROR("Created service");
		    }

		    if(!_srvc_release_ball[robotIdx].exists())
			{
				std::string strRelBall = ownerName + "/" +
						WorldModelInterface::s_release_own_ball_possession;
				ros::service::waitForService(strRelBall);

				_srvc_release_ball[robotIdx] = m_nh.serviceClient<
						worldModel::release_own_ball_possession>(strRelBall, USE_SERVICE_PERSISTENCY);
				TRACE_ERROR("Created service");
			}

		    if(!_srvc_CameraBallPossession[robotIdx].exists())
		    {
		    	std::string strCameraBallPossession = ownerName + "/" +
		    			WorldModelInterface::s_set_own_camera_ball_possession;
		    	ros::service::waitForService(strCameraBallPossession);

		    	_srvc_CameraBallPossession[robotIdx] = m_nh.serviceClient<
		    			worldModel::set_own_camera_ball_possession>(strCameraBallPossession, USE_SERVICE_PERSISTENCY);
		    	TRACE_ERROR("Created service");
		    }

		    // calling service set_own_ball_location for this robot
		    worldModel::overrule_ball_location srv_ball_location;
		    srv_ball_location.request.robotID = robotID.data;
		    // determine ball position
		    // no need to transform from absolute coordinates to robot coordinates since it is a relative position
 	        Position2D robot_pos(m_worldinfo.robots[irobot].x, m_worldinfo.robots[irobot].y, m_worldinfo.robots[irobot].orient);
		    TRACE("irobot=%d robot_pos: x=%6.2f y=%6.2f", irobot, robot_pos.x, robot_pos.y);
 	        Position2D relative_ball_pos(m_worldinfo.ball.x, m_worldinfo.ball.y, 0);
 	        relative_ball_pos.transform_fcs2rcs(robot_pos);

		    TRACE("relative_ball_pos x=%6.2f y=%6.2f", relative_ball_pos.x,
				    relative_ball_pos.y);
		    if (m_worldinfo.ball.owner.size()) {
			    srv_ball_location.request.robotID = robotID.data;
		    }

		    float radiusBall = relative_ball_pos.size();
		    if(radiusBall < _VIEW_DISTANCE_ROBOT)
		    {
		    	/* Only overrule ball location if robot can see it */
		    	srv_ball_location.request.positionX = m_worldinfo.ball.x;
		    	srv_ball_location.request.positionY = m_worldinfo.ball.y;
		    	srv_ball_location.request.velocityX = m_worldinfo.ball.vx;
		    	srv_ball_location.request.velocityY = m_worldinfo.ball.vy;
                if (!playing_left_to_right) // transformACS2FCS: mirror image
                {
                    srv_ball_location.request.positionX = -m_worldinfo.ball.x;
                    srv_ball_location.request.positionY = -m_worldinfo.ball.y;
                    srv_ball_location.request.velocityX = -m_worldinfo.ball.vx;
                    srv_ball_location.request.velocityY = -m_worldinfo.ball.vy;
                }
		    	srv_ball_location.request.positionZ = m_worldinfo.ball.z;
		    	srv_ball_location.request.velocityZ = m_worldinfo.ball.vz;
		    	srv_ball_location.request.robotID = robotID.data;
		    	srv_ball_location.request.overruleTimeInSeconds = 1.0;

		    	TRACE("set_ball_location x=%6.2f y=%6.2f robotID=%d",
				    srv_ball_location.request.positionX,
				    srv_ball_location.request.positionY,
				    int(srv_ball_location.request.robotID));

		    	if (!_srvc_ball_location[robotIdx].call(srv_ball_location))
		    	{
					TRACE_ERROR("Service request srv_ball_location failed");
					std::string ball_service_name = ownerName + "/"
						   + WorldModelInterface::s_set_overrule_ball_location;
					ros::service::waitForService(ball_service_name);

					_srvc_ball_location[robotIdx] = m_nh.serviceClient<
						 worldModel::overrule_ball_location>(ball_service_name,
						 USE_SERVICE_PERSISTENCY);
					TRACE_ERROR("Created service");
		    	}
		    }

			TRACE("m_worldinfo.ball.owner=%s", m_worldinfo.ball.owner.c_str());
				
		    /* Update own ball possession when needed */
			if((m_worldinfo.ball.owner.size()) && (ownerName.find(m_worldinfo.ball.owner) != std::string::npos))
			{
				/* New robot claimed ball in simball */
				robotHasBallID = robotIdx;
				robotHasBall = true;

				worldModel::claim_own_ball_possession srvClaimBall;
				worldModel::set_own_camera_ball_possession srvCamPossession;
				TRACE("claiming ball, robot=%d, srvidx=%d", int(robotID.data), robotHasBallID);

				srvCamPossession.request.ballInfrontOfRobot = true;

				if(!_srvc_CameraBallPossession[robotIdx].call(srvCamPossession))
				{
					TRACE_ERROR("Failed to claim camera ball in worldModel for robotID %d",
							robotHasBallID);
					std::string strCamPoss = ownerName + "/" +
							WorldModelInterface::s_set_own_camera_ball_possession;
					ros::service::waitForService(strCamPoss);

					_srvc_CameraBallPossession[robotIdx] = m_nh.serviceClient<
							worldModel::set_own_camera_ball_possession>(strCamPoss, USE_SERVICE_PERSISTENCY);
					TRACE_ERROR("Created service");
				}

				if(!_srvc_claim_ball[robotIdx].call(srvClaimBall))
				{
					TRACE_ERROR("Failed to claim ball in worldModel for robotID %d",
							robotHasBallID);
					std::string strClaimBall = ownerName + "/" +
							WorldModelInterface::s_claim_own_ball_possession;
					ros::service::waitForService(strClaimBall);

					_srvc_claim_ball[robotIdx] = m_nh.serviceClient<
							worldModel::claim_own_ball_possession>(strClaimBall, USE_SERVICE_PERSISTENCY);
					TRACE_ERROR("Created service");
				}
			}

			if ((robotHasBall) && (m_worldinfo.ball.owner.size() == 0))
			{
				/* Release the claimed ball */
				robotHasBall = false;

				worldModel::set_own_camera_ball_possession srvCamPossession;
				worldModel::release_own_ball_possession srvRelBall;

				TRACE("release ball, robot=%d, srvidx=%d", int(robotID.data), robotHasBallID);

				srvCamPossession.request.ballInfrontOfRobot = false;

				if(!_srvc_CameraBallPossession[robotIdx].call(srvCamPossession))
				{
					TRACE_ERROR("Failed to claim camera ball in worldModel for robotID %d",
							robotHasBallID);
					std::string strCamPoss = ownerName + "/" +
							WorldModelInterface::s_set_own_camera_ball_possession;
					ros::service::waitForService(strCamPoss);

					_srvc_CameraBallPossession[robotIdx] = m_nh.serviceClient<
							worldModel::set_own_camera_ball_possession>(strCamPoss, USE_SERVICE_PERSISTENCY);
					TRACE_ERROR("Created service");
				}

				if(!_srvc_release_ball[robotHasBallID].call(srvRelBall))
				{
					TRACE_ERROR("Failed to release ball in worldModel for robotID %d",
							robotHasBallID);
					std::string strRelBall = ownerName + "/" +
							WorldModelInterface::s_release_own_ball_possession;
					ros::service::waitForService(strRelBall);

					_srvc_release_ball[robotIdx] = m_nh.serviceClient<
							worldModel::release_own_ball_possession>(strRelBall, USE_SERVICE_PERSISTENCY);
					TRACE_ERROR("Created service");
				}

			}

		    /* Get ball possession from worldModel */
//		    worldModel::get_ball_possession srv_ball_possession;
//		    if (!_srvc_ball_possession[robotIdx].call(srv_ball_possession))
//		    {
//		        TRACE_ERROR("Service request ball possession failed");
//		        std::string possession_service_name = ownerName + "/"
//                        + WorldModelInterface::s_get_ball_possession;
//                ros::service::waitForService(possession_service_name);
//
//                _srvc_ball_possession[robotIdx] = m_nh.serviceClient<
//                        worldModel::get_ball_possession>(possession_service_name,
//                        USE_SERVICE_PERSISTENCY);
//                TRACE_ERROR("Created service");
//		    }
				
		    int robotIdx2 = 0;

		    // set_obstacle_location
	        for (int irobot2 = 0; irobot2 < (int) m_worldinfo.robots.size(); ++irobot2) 
	        {
	            std::string myname = m_worldinfo.robots[irobot2].name;
	            robotIdx2 = myname[5] - '0'; // assume string "robotN"

	            // inform our position to enemy team
		        if (m_worldinfo.robots[irobot2].team != m_teamname)
		        {
                    std::string obst_service_name = "/" + m_worldinfo.robots[irobot2].team + "/" + m_worldinfo.robots[irobot2].name + "/"
                            + WorldModelInterface::s_set_overrule_obstacle_location;
		        	if(!_srvc_obstacle_enemy_location[robotIdx2].exists())
                    {
                        ros::service::waitForService(obst_service_name);
		                TRACE("creating cross-team obstacle service on %s", obst_service_name.c_str());
				    
                        _srvc_obstacle_enemy_location[robotIdx2] = m_nh.serviceClient<
                                worldModel::overrule_obstacle_location>(obst_service_name,
                                USE_SERVICE_PERSISTENCY);
                        TRACE_ERROR("Created service");
                    }
                                
	                worldModel::overrule_obstacle_location srv_obst_location;
         	        Position2D obst_pos(m_worldinfo.robots[irobot].x, m_worldinfo.robots[irobot].y, m_worldinfo.robots[irobot].orient);
         	        Position2D other_robot_pos(m_worldinfo.robots[irobot2].x, m_worldinfo.robots[irobot2].y, m_worldinfo.robots[irobot2].orient);
         	        playing_left_to_right = (m_worldinfo.robots[irobot2].team == "teamA");
         	        obst_pos.transform_acs2fcs(playing_left_to_right);
        		    TRACE("obst_pos FCS x=%6.2f y=%6.2f phi=%6.2f", obst_pos.x, obst_pos.y, obst_pos.phi);
         	        //obst_pos = obst_pos.transform_fcs2rcs(other_robot_pos);
        		    //TRACE("obst_pos RCS x=%6.2f y=%6.2f phi=%6.2f", obst_pos.x, obst_pos.y, obst_pos.phi);
	                srv_obst_location.request.positionX.push_back(obst_pos.x);
	                srv_obst_location.request.positionY.push_back(obst_pos.y);
	                srv_obst_location.request.velocityX.push_back(0.0);
	                srv_obst_location.request.velocityY.push_back(0.0);
	                srv_obst_location.request.overruleTimeInSeconds = 1.0;
	                srv_obst_location.request.nrObstacles = srv_obst_location.request.positionX.size();


        		    TRACE("set_obst_location irobot2=%d x=%6.2f y=%6.2f", obst_pos.x, obst_pos.y, irobot2);
		            if (!_srvc_obstacle_enemy_location[robotIdx2].call(srv_obst_location))
		            {
		                TRACE_ERROR("Service request _srvc_obstacle_enemy_location failed");
		                ros::service::waitForService(obst_service_name);
		                TRACE("creating cross-team obstacle service on %s", obst_service_name.c_str());

                        _srvc_obstacle_enemy_location[robotIdx2] = m_nh.serviceClient<
                                worldModel::overrule_obstacle_location>(obst_service_name,
                                USE_SERVICE_PERSISTENCY);
                        TRACE_ERROR("Created service");
		            }


		        }
	        }
        }
	}
}

} // end of namespace simulator

int main(int argc, char **argv) {
    TRACE("initializing");
	                
	ros::init(argc, argv, "simteam");
	ros::NodeHandlePtr nh_;
	nh_.reset(new ros::NodeHandle);

	std::string teamname = std::string("team") + getTeamChar();
    TRACE("teamname=%s", teamname.c_str());

	// construct and spin
	simulator::SimTeam *team = new simulator::SimTeam(ros::NodeHandle("simteam"),
			teamname);
	ros::Rate updateRate(10);
	while (ros::ok()) {
		ros::spinOnce();
		updateRate.sleep();
	}

	delete team;

	return 0;
}

