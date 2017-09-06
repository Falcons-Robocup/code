 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: jfeitsma
 * Creation: 2014-04-07
 *
 * Redesigning simulator from Niels Koenraad.
 * Separate processes
 *   visualize
 *   simrobot N
 *   simball
 *
 * This is the simrobot process.
 *
 * CHANGELOG
 * 2014-10-16 cut out wheel velocity and OROCOS layer, preparing for transition
 *            to firmware API from Prabu
 * 2015-05-06 SAAR Added teleport functionality
 *
 */

#include <simulator/Position.h>
#include <simulator/WorldInfo.h>
#include <simulator/simulatorConfig.h>
#include <dynamic_reconfigure/server.h>
#include "rosMsgs/t_robotspeed.h"
#include "rosMsgs/t_sim_position.h"
#include "worldModel/overrule_own_location.h"
#include "WorldModelNames.h"
#include <sstream>
#include <ros/ros.h>
#include <math.h>

#include "FalconsCommon.h"
#include <cDiagnosticsEvents.hpp>


#define SQR(x) ((x)*(x))

namespace simulator
{

// DECLARATION

class SimRobot
{
public:
	SimRobot(ros::NodeHandle *nh, std::string const &teamname, std::string const &robotname);
	void update(double dt);
private:
	void velocityCallback(const rosMsgs::t_robotspeed::ConstPtr& vel);
	void teleportCallback(const rosMsgs::t_sim_position::ConstPtr& vel);
	void worldinfoCallback(const WorldInfoConstPtr& worldinfo);
	void configCallback(simulatorConfig &config, uint32_t level);
	float generateNoise(float max_value);
    void determine_initial_position();
    
	ros::NodeHandle * m_nh;
	// pos&vel in the world
	Position2D        m_pos; // ACS
	Velocity2D        m_vel; // RCS on input, transform to ACS during update()
	Velocity2D        pp_vel_rcs;
	std::string       m_teamname;
	std::string       m_robotname;
	int               m_robotnum;
	int               m_iteration;
	simulatorConfig    m_config;
	bool              m_active; // once WorldInfo is received, the bot can place itself without colliding. Then update() will also become active

	WorldInfo         m_worldinfo;
	ros::Subscriber   m_config_sub;
	ros::Subscriber   m_velocity_sub;
	ros::Subscriber   m_teleport_sub;
	ros::Subscriber   m_worldinfo_sub;
	ros::Publisher    m_sim_position_pub;
	ros::ServiceClient m_own_position_srv;

}; // class SimRobot



// IMPLEMENTATIONS

SimRobot::SimRobot(ros::NodeHandle * nh, const std::string &teamname, const std::string &robotname) :
	m_nh(nh), m_teamname(teamname), m_robotname(robotname), m_iteration(0), m_active(false)
{
	TRACEF("SimRobot constructor, teamname=%s, robotname=%s", m_teamname.c_str(), m_robotname.c_str());

	// subscribe to dynamic settings
	m_velocity_sub = m_nh->subscribe("g_robotspeed", 100, &SimRobot::velocityCallback, this);
	m_teleport_sub = m_nh->subscribe("g_sim_position", 100, &SimRobot::teleportCallback, this);
	m_worldinfo_sub = m_nh->subscribe("/simulator/worldinfo", 100, &SimRobot::worldinfoCallback, this);
	m_sim_position_pub = m_nh->advertise<rosMsgs::t_sim_position>("g_sim_position", 100);

	/* Subscribe to set own position */
	std::string service_name = WorldModelInterface::s_set_overrule_own_location;
	ros::service::waitForService(service_name);
	m_own_position_srv = m_nh->serviceClient<worldModel::overrule_own_location>(service_name, USE_SERVICE_PERSISTENCY);

	/* Dynamic reconfiguration */
	dynamic_reconfigure::Server<simulator::simulatorConfig> srv(*m_nh);
	dynamic_reconfigure::Server<simulator::simulatorConfig>::CallbackType cb;
	cb = boost::bind(&SimRobot::configCallback, this, _1, _2);
	srv.setCallback(cb);

    m_robotnum = int(m_robotname[5] - '0');
    determine_initial_position(); // will set robot to active
	TRACE("initial position %s", m_pos.tostr());
}

float SimRobot::generateNoise(float max_value)
{
    return (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * max_value;
}


void SimRobot::determine_initial_position()
{
    // positions in ACS
    if (m_teamname == "teamB")
    {
        // assume playing left to right
        switch (m_robotnum)
        {
          case 1: 
            m_pos.x = -3.0;
            m_pos.y = -5.0;
            break;
          case 2: 
            m_pos.x =  0.0;
            m_pos.y = -5.0;
            break;
          case 3: 
            m_pos.x =  3.0;
            m_pos.y = -5.0;
            break;
          case 4: 
            m_pos.x = -3.0;
            m_pos.y = -2.0;
            break;
          case 5: 
            m_pos.x =  0.0;
            m_pos.y = -2.0;
            break;
          case 6: 
            m_pos.x =  3.0;
            m_pos.y = -2.0;
            break;
          default:
            TRACE_ERROR("invalid robotnum %d", m_robotnum);
            break;
        }
        m_pos.phi = 0.0;
    }
    if (m_teamname == "teamA")
    {
        // assume playing right to left, 
        // note that FCS from teamB will be rotated 180 degrees w.r.t. FCS from teamA
        // but simulator does not distinguish (ACS)
        switch (m_robotnum)
        {
          case 1: 
            m_pos.x =  3.0;
            m_pos.y =  5.0;
            break;
          case 2: 
            m_pos.x =  0.0;
            m_pos.y =  5.0;
            break;
          case 3: 
            m_pos.x = -3.0;
            m_pos.y =  5.0;
            break;
          case 4: 
            m_pos.x =  3.0;
            m_pos.y =  2.0;
            break;
          case 5: 
            m_pos.x =  0.0;
            m_pos.y =  2.0;
            break;
          case 6: 
            m_pos.x = -3.0;
            m_pos.y =  2.0;
            break;
          default:
            TRACE_ERROR("invalid robotnum %d", m_robotnum);
            break;
        }
        m_pos.phi = 3.15;
    }
    m_active = true;
}

void SimRobot::worldinfoCallback(const WorldInfoConstPtr& worldinfo) {
	m_worldinfo = *worldinfo;
	// replaced dynamic spawning (which was problematic for multi-team simulation)
	// by fixed location spawning in determine_initial_position()
} // void SimRobot::worldinfoCallback(const WorldInfoConstPtr& worldinfo)

void SimRobot::velocityCallback(const rosMsgs::t_robotspeed::ConstPtr& vel) {
	// speed in RCS, will be transformed to ACS just before updating m_pos (which is in FCS)
	m_vel = Velocity2D(vel->vx, vel->vy, vel->vphi);
	TRACE("velocityCallback rcs_%s", m_vel.tostr());
	pp_vel_rcs = Velocity2D(vel->vx, vel->vy, vel->vphi);
} // void SimRobot::velocityCallback(const rosMsgs::t_robotspeed::ConstPtr& vel)

void SimRobot::teleportCallback(const rosMsgs::t_sim_position::ConstPtr& pos) {
	// Set the position of the robot in x, y, phi
	m_pos = Position2D(pos->x, pos->y, pos->orient);
	TRACE("teleportCallback %s", m_pos.tostr());
} // void SimRobot::teleportCallback(const rosMsgs::t_sim_position::ConstPtr& pos)

void SimRobot::update(double dt) {
	if (m_active == false) {
		TRACE("update skip - not yet active");
		return;
	}

	TRACE("update start");
    // TODO get rid of worldinfo. I'd rather have WorldModel offer a service 
    // for things like field size, edges.

	if ((m_worldinfo.xsize < 1) || (m_worldinfo.ysize < 1)) {
		if (m_iteration > 20) {
			ROS_ERROR("WorldInfo does not seem to be present - is process visualize running?");
		}
		return;
	}

	TRACE("current position (ACS)    : %s", m_pos.tostr());
	//TRACE("current velocity (RCS)    : %s", m_vel.tostr());

	bool playing_left_to_right = (m_teamname == "teamA");
	// transform speed from robot coordinates to field coordinates
	m_pos.transform_acs2fcs(playing_left_to_right); // needed for next transformation
	//TRACE("current position (FCS)    : %s", m_pos.tostr());
	m_vel.transform_rcs2fcs(m_pos); // vx,vy only
	//TRACE("current velocity (FCS)    : %s", m_vel.tostr());
	m_pos.transform_fcs2acs(playing_left_to_right); // transform back to ACS
	// transform to ACS
	m_vel.transform_fcs2acs(playing_left_to_right);

	// update position
    m_pos.update(m_vel, dt);
	TRACE("new     position (ACS)    : %s", m_pos.tostr());

	// Clamp to screen size
	double tol = 0.1; // TODO: half of robot radius - put in WorldInfo?
	double xmin = -m_worldinfo.xsize/2 - m_worldinfo.edge_x + tol;
	double xmax =  m_worldinfo.xsize/2 + m_worldinfo.edge_x - tol;
	double ymin = -m_worldinfo.ysize/2 - m_worldinfo.edge_y + tol;
	double ymax =  m_worldinfo.ysize/2 + m_worldinfo.edge_y - tol;
	if ((m_pos.x < xmin) || (m_pos.x > xmax) || (m_pos.y < ymin) || (m_pos.y > ymax)) {
		TRACE("Oh no! I hit the wall! ");
		ROS_WARN("Oh no! I hit the wall! ");
		m_pos.x = std::min(std::max(m_pos.x, xmin), xmax);
		m_pos.y = std::min(std::max(m_pos.y, ymin), ymax);
	}

	// publish current simulated position (no velocity)
	rosMsgs::t_sim_position local_pos;
	local_pos.x = m_pos.x;
	local_pos.y = m_pos.y;
	local_pos.orient = m_pos.phi;
	TRACE("publish %s", m_pos.tostr());

#ifdef USE_SIMULATOR_PATHPLANNING_WORKAROUND
	// JFEI 2014-11-20 workaround for WorldModel being broken:
	// also feedback current position to pathplanning
	local_pos.vx = m_vel.x;
	local_pos.vy = m_vel.y;
	local_pos.vphi = m_vel.phi;
	TRACE("publish %s", m_vel.tostr());
#endif

	m_sim_position_pub.publish(local_pos);

	// call WorldModel service to set current position
	// regardless of USE_SIMULATOR_PATHPLANNING_WORKAROUND
	// Add some noise to the new coordinates to imitate WorldSensing camera
	Position2D pos_fcs = m_pos;
	Velocity2D vel_fcs = pp_vel_rcs;
	pos_fcs.transform_acs2fcs(playing_left_to_right);
	vel_fcs.transform_rcs2fcs(m_pos);
	worldModel::overrule_own_location srv_own_location;
	srv_own_location.request.x = pos_fcs.x; // + generateNoise(0.01); EKPC -- Removed noise. It has no added value in simulation.
	srv_own_location.request.y = pos_fcs.y; // + generateNoise(0.01);
	srv_own_location.request.theta = pos_fcs.phi; // + generateNoise(0.001);
	srv_own_location.request.vx = vel_fcs.x;
	srv_own_location.request.vy = vel_fcs.y;
	srv_own_location.request.vtheta = vel_fcs.phi;
	srv_own_location.request.overruleTimeInSeconds = 1.0; //1 second
	// WorldModel by itself will estimate velocities
	if (!m_own_position_srv.call(srv_own_location))
	{
		TRACE("error in srv_own_location call");
		ROS_ERROR("Service request srv_own_location failed");

		/* Subscribe to set own position */
		std::string service_name = WorldModelInterface::s_set_overrule_own_location;
		ros::service::waitForService(service_name);
		m_own_position_srv = m_nh->serviceClient<worldModel::overrule_own_location>(service_name, USE_SERVICE_PERSISTENCY);
	}

	m_iteration++;
	TRACE("update end");

	// zero velocity to prevent drift
	m_vel = Velocity2D(0, 0, 0);

} // void SimRobot::update(double dt)

void SimRobot::configCallback(simulator::simulatorConfig &config, uint32_t level)
{
   m_config = config;
	TRACE("SimRobot::configCallback x=%f y=%f", m_config.pixels_per_meter_x, m_config.pixels_per_meter_y);

} // end configCallback()

} // end of namespace simulator





int main(int argc, char **argv)
{
	// JFEI not anymore via args, but via env
	// see: http://timmel.no-ip.biz:8000/wiki/DesignRules#Environment
	
	// Startup arg checking
	std::string teamname = "team" + std::string(1, getTeamChar());
    int robotNumber = getRobotNumber();
    std::string robotname = "robot" + toStr(robotNumber);
    
	// check if teamname is either teamA or teamB
	if (!(teamname == "teamA" || teamname == "teamB")) {
		ROS_ERROR("Team name must be either teamA or teamB.");
		return -1;
	}

	// check if robotname is indeed a number in rang 1 - 6
	if (robotNumber < 1 || robotNumber > 6) {
		ROS_ERROR("Robot name must be in range 1 - 6");
		return -1;
	}
	// End arg checking

	ros::init(argc, argv, "simrobot");
	ros::NodeHandle nh;

	// construct and spin
	simulator::SimRobot *bot = new simulator::SimRobot(&nh, teamname, robotname);

	ros::Rate loop_rate(MOTION_FREQUENCY);
	while (ros::ok()) {
		// make sure WorldInfo is present
		// so process 'visualize' always needs to be loaded first
		ros::spinOnce();
		TRACE("spinned");

		bot->update(1.0 / MOTION_FREQUENCY); // JFEI 2014-10-30 was 0.015

		// make sure loop runs on specified frequency
		TRACE("sleeping");
        loop_rate.sleep();
	}

	return 0;
}

