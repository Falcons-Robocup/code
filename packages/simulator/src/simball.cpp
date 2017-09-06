 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: jfeitsma
 * Creation: 2014-04-25
 *
 * Redesigning simulator from Niels Koenraad.
 * Separate processes
 *   visualize
 *   simrobot N
 *   simball
 *
 * This is the simball process.
 * TODO: allow for more balls? Does WorldModel+TeamPlay see a use case for this?
 *
 * CHANGELOG
 *  2014-08-07 JFEI change coordinate system convention according to PL decision wk1428
 *  2015-04-08 TKOV Try to fix vanishing balls
 *  2017-06-06 MVEH increase shootpower magic number to have pass OK, shot acceptable
 */

#include <simulator/BallInfo.h>
#include <simulator/WorldInfo.h>
#include <simulator/simulatorConfig.h>
#include <simulator/BallPossessionSim.h>
#include <simulator/Shoot.h>
#include <simulator/TeleportBall.h>
#include "ext/robotSimNames.h"
#include <dynamic_reconfigure/server.h>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <math.h>

#include "rosMsgs/BallPossession.h"
#include "FalconsCommon.h" // TRACE, Vector2D and Circle

#define SQR(x) ((x)*(x))


namespace simulator
{

// auxiliaries for sorting obstacles during collision handling
struct Obstacle
{
	Circle posr;
	Vector2D speed;
	Obstacle(Circle c) : posr(c), speed(Vector2D(0.0, 0.0)) {};
	Obstacle(Circle c, Vector2D s) : posr(c), speed(s) {};
};
struct compare_distance_from_ball
{
	Vector2D mypos;
	double myradius;
	compare_distance_from_ball(Vector2D p, double r) : mypos(p), myradius(r) {};
	inline bool operator()(Obstacle const &o1, Obstacle const &o2)
	{
		double d1 = vectorsize(o1.posr.pos - mypos) - o1.posr.r - myradius;
		double d2 = vectorsize(o2.posr.pos - mypos) - o2.posr.r - myradius;
		return d1 < d2;
	}
};




// DECLARATION

class SimBall
{
public:
	SimBall(const ros::NodeHandle& nh);
	void update(double dt);
private:
	void worldinfoCallback(const WorldInfoConstPtr& worldinfo);
	void configCallback(simulatorConfig &config, uint32_t level);
	void ballPossessionCallback(const BallPossessionSimConstPtr ballPossession);
    bool shootCallback(simulator::Shoot::Request&, simulator::Shoot::Response&);
    bool teleportCallback(simulator::TeleportBall::Request&, simulator::TeleportBall::Response&);
	void resolveCollision(const Circle &other, Vector2D const &, const double dt);
	void stepAndPublish(double dt);

	ros::NodeHandle   m_nh;
	// pos&vel in the world
	BallInfo          m_ballinfo;
	int               m_iteration;
	bool              m_active; // once WorldInfo is received, the ball can place itself without colliding. Then update() will also become active
	float             m_shoot_velocity; // set by service interrupt, handled in next update
	int               m_cooldown; // workaround counter for preventing an escaping ball being immediately re-caught by its owner
	WorldInfo         m_worldinfo;
	std::vector<Obstacle> m_obstacles;
	simulatorConfig    m_config;
	ros::Subscriber   m_worldinfo_sub;
	ros::Subscriber   m_BallPossession_sub;
	ros::Publisher    m_ballinfo_pub;
    ros::ServiceServer m_shoot_srv;
    ros::ServiceServer m_teleport_srv;
    uint8_t            m_BallPossessionType;
    uint8_t            m_BallPossessionRobotID;
    char               m_BallPossessionTeam;

}; // class SimBall



// IMPLEMENTATIONS

SimBall::SimBall(const ros::NodeHandle& nh) :
		m_nh(nh), m_iteration(0), m_active(false), m_shoot_velocity(0.0), m_cooldown(0)
{
	TRACE("SimBall constructor");

	m_ballinfo_pub = m_nh.advertise<
			simulator::BallInfo>(
			"/simulator/ball", 100);

	m_worldinfo_sub = m_nh.subscribe(RobotsimInterface::S_SIM_WORLDINFO, 100,
			&SimBall::worldinfoCallback, this);

	m_BallPossession_sub = m_nh.subscribe(RobotsimInterface::S_SIM_BALLPOSSESSION, 100,
	        &SimBall::ballPossessionCallback, this);

	/* Dynamic reconfiguration */
	ros::NodeHandle tmp_nh(RobotsimNodeNames::ROBOTSIM_SIMCONFIG);
	dynamic_reconfigure::Server<simulator::simulatorConfig> srv(tmp_nh);
	dynamic_reconfigure::Server<simulator::simulatorConfig>::CallbackType cb;
	cb = boost::bind(&SimBall::configCallback, this, _1, _2);
	srv.setCallback(cb);

	// shoot trigger (TODO: link to actual robot shoot trigger)
	m_shoot_srv = m_nh.advertiseService(RobotsimInterface::S_SIM_BALl_SHOOT, &SimBall::shootCallback, this);
	m_teleport_srv = m_nh.advertiseService(RobotsimInterface::S_SIM_BALL_TELEPORT, &SimBall::teleportCallback, this);
	
    m_BallPossessionType = 0;
    m_BallPossessionRobotID = 0;
    m_BallPossessionTeam = '?';
	
}


void SimBall::worldinfoCallback(const WorldInfoConstPtr& worldinfo)
{
	m_worldinfo = *worldinfo;
	// determine starting position
	if (false == m_active)
	{
		TRACE("placing ball");
		// (0,0) at field center
		float x = 0;
		float y = 0;
		float w = m_config.edge_x + m_config.field_size_x/2.0;
		float h = m_config.edge_y + m_config.field_size_y/2.0;

		// we could use a obstacle-avoiding place loop as in SimRobot
		// but we do not care that much anyway. Put it at center of the field.
		m_ballinfo.x = x;
		m_ballinfo.y = y;

		// initialize world as a list of static obstacles (need doubles)
		m_obstacles.push_back(Circle(     0, -h-1e8, 1e8));
		m_obstacles.push_back(Circle(     0,  h+1e8, 1e8));
		m_obstacles.push_back(Circle(-w-1e8,      0, 1e8));
		m_obstacles.push_back(Circle( w+1e8,      0, 1e8));
		// goal lines: array of circles
		float g = m_config.goal_size / 2;
		for (int it = 0; it < 15; it++)
		{
			m_obstacles.push_back(Circle(-g, -h+0.1*it, 0.1));
			m_obstacles.push_back(Circle(-g,  h-0.1*it, 0.1));
			m_obstacles.push_back(Circle( g, -h+0.1*it, 0.1));
			m_obstacles.push_back(Circle( g,  h-0.1*it, 0.1));
		}
		// hardcoded: goal backside (depth=60cm)
		h = m_config.field_size_y/2.0;
		float d = 0.8;
		x = -g;
		while (x < g)
		{
			m_obstacles.push_back(Circle( x,  h+d, 0.2));
			m_obstacles.push_back(Circle( x, -h-d, 0.2));
		    x += 0.1;
	    }
	}
	// set active flag
	m_active = true;
}



void SimBall::configCallback(simulator::simulatorConfig &config, uint32_t level)
{
    m_config = config;
    TRACE("SimBall::configCallback x=%f y=%f", m_config.pixels_per_meter_x, m_config.pixels_per_meter_y);

} // end configCallback()


bool SimBall::shootCallback(simulator::Shoot::Request& req, simulator::Shoot::Response&)
{

	TRACE("SimBall::shootCallback velocity=%f", req.velocity);
	// check if ball is owned by a robot, otherwise it cannot be shot
	if (m_ballinfo.owner.size())
	{
		// raise shoot flag, to be handled during update()
    	// MVEH: was 1.8, way too soft when using teamplay shoot to nearest team mate. 
		// 4 is still a bit too soft for passing, ball is almost at 0 velocity when it reaches next player (test with 3 robots)
		// however shot at goal at full power is now already crazy hard, don't want to increase it further.
		// Seems the max shotpower is too high in the simulator
		m_shoot_velocity = req.velocity * 4; 

		return true;
	}
	return false;
}

bool SimBall::teleportCallback(simulator::TeleportBall::Request& req, simulator::TeleportBall::Response&)
{

	TRACE("SimBall::teleportCallback posx=%f posy=%f velx=%f vely=%f", req.x, req.y, req.vx, req.vy);
	// release ball owner
	if( (!isnan(req.x)) && (!isnan(req.y)) && (!isnan(req.vx)) && (!isnan(req.vy)) )
	{
        m_ballinfo.owner.clear();
        m_cooldown = 1; // don't stall too many iterations
        m_shoot_velocity = 0.0;
        m_ballinfo.x = req.x;
        m_ballinfo.y = req.y;
        m_ballinfo.vx = req.vx;
        m_ballinfo.vy = req.vy;
        m_BallPossessionRobotID = 0;
        m_BallPossessionType = rosMsgs::BallPossession::TYPE_FIELD;
        m_BallPossessionTeam = '?';
	}

	return true;
}


void SimBall::resolveCollision(Circle const &other, Vector2D const &otherspeed, const double dt)
// adapted from: http://stackoverflow.com/questions/345838/ball-to-ball-collision-detection-and-handling
// and en.wikipedia.org/wiki/Elastic_collision, bottom section
{
	TRACE("resolveCollision with circle (x=%6.2f y=%6.2f r=%6.2f vx=%6.2f vy=%6.2f)", other.pos.x, other.pos.y, other.r, otherspeed.x, otherspeed.y);

	Vector2D position(m_ballinfo.x, m_ballinfo.y);
	Vector2D otherposition(other.pos.x, other.pos.y);
	Vector2D posdelta = position - otherposition;
	double d = posdelta.size();
	TRACE("posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, d);
    if (d < 0.01)
    {
    	d = 0.01; // prevent div by zero
    	TRACE("wutwutwut??? In the Butt!");
    }

    // mass quantities
    double m1 = 1.0;
    double m2 = 1e8; // assume 2nd object is immovable by ball (either wall or robot)

    // velocity difference
	Vector2D velcurr = Vector2D(m_ballinfo.vx, m_ballinfo.vy);
	Vector2D veldelta = velcurr - otherspeed;
	double vd = veldelta.size();
	if (vd < 0.01)
	{
    	vd = 0.01; // prevent div by zero
	}
	
	// the ball and other object are already overlapping... solve for the exact moment of collision
	//double dt = 0.015; // value does not matter for this solving, I think
	double C = SQR(vectorsize(posdelta)) - SQR(m_ballinfo.size/2 + other.r);
	double A = SQR(dt * vd);
	double B = -2 * dt * vectordot(veldelta, posdelta);
	double D = B*B - 4*A*C;
	double a = (-B + sqrt(D)) / (2*A); // amount of incursion is now solved
	// correct positions for the incursion
	position = position - a * velcurr * dt;
	otherposition = otherposition - a * otherspeed * dt;
	posdelta = position - otherposition;
	d = posdelta.size();
	TRACE("incursion C=%f A=%f B=%f a=%f", C, A, B, a);
	TRACE("2nd posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, d);

	// calculate new speed
	Vector2D new_velocity = velcurr - (2*m2 / (m1+m2)) * vectordot(veldelta, posdelta) / (d*d) * posdelta;
	TRACE("new_velocity vx=%6.2f vy=%6.2f", new_velocity.x, new_velocity.y);
    m_ballinfo.vx = new_velocity.x * m_config.ball_bump_factor;
    m_ballinfo.vy = new_velocity.y * m_config.ball_bump_factor;

	// correct ball position again for the incursion, but not yet doing the v*dt step because that one will be handled by stepAndPublish
	Vector2D newposition = position + a * new_velocity * dt;
	TRACE("newposition x=%6.2f y=%6.2f", newposition.x, newposition.y);
    m_ballinfo.x = newposition.x;
    m_ballinfo.y = newposition.y;
}

void SimBall::stepAndPublish(double dt)
{
	m_ballinfo.x += m_ballinfo.vx * dt;
	m_ballinfo.y += m_ballinfo.vy * dt;

	// TODO need bounsimulatordary checks? should have been resolved (collisions)
	m_ballinfo.vx *= m_config.ball_friction;
	m_ballinfo.vy *= m_config.ball_friction;
	if (m_ballinfo.vx*m_ballinfo.vx + m_ballinfo.vy*m_ballinfo.vy < m_config.ball_friction_eps*m_config.ball_friction_eps)
	{
		m_ballinfo.vx = 0;
		m_ballinfo.vy = 0;
	}
	m_ballinfo.size = m_config.ball_size;
	TRACE("SimBall::stepAndPublish pos(%6.2f,%6.2f) vel(%6.2f,%6.2f)", m_ballinfo.x, m_ballinfo.y, m_ballinfo.vx, m_ballinfo.vy);

	if ( (!isnan(m_ballinfo.x)) &&
	     (!isnan(m_ballinfo.y)) &&
	     (!isnan(m_ballinfo.z)) &&
	     (!isnan(m_ballinfo.vx)) &&
	     (!isnan(m_ballinfo.vy)) &&
	     (!isnan(m_ballinfo.vz)) &&
	     (!isnan(m_ballinfo.size)) )
	{
        BallInfo result = m_ballinfo;
        float BALLNOISE = 0.0;
	    // add some noise
	    result.x += ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * BALLNOISE) - BALLNOISE*0.5;
	    result.y += ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * BALLNOISE) - BALLNOISE*0.5;
	    m_ballinfo_pub.publish(result);
	}
}


void SimBall::update(double dt)
{
	TRACE("SimBall::update");
	if ((m_worldinfo.xsize < 1.0) || (m_worldinfo.ysize < 1.0))
	{
		if (m_iteration > 20)
		{
			ROS_ERROR("WorldInfo does not seem to be present - is process visualize running?");
		}
		return;
	}

	// handle shoot request (service)
	if (m_shoot_velocity > 0.00001)
	{
		TRACE("SimBall::update shoot from owner %s with velocity %f", m_ballinfo.owner.c_str(), m_shoot_velocity);
		// determine directed speed
		for (size_t ir=0; ir < m_worldinfo.robots.size(); ++ir)
		{
			if (m_ballinfo.owner ==  m_worldinfo.robots[ir].team + "/" + m_worldinfo.robots[ir].name)
			{
				m_ballinfo.vx =  cos(m_worldinfo.robots[ir].orient) * m_shoot_velocity;
				m_ballinfo.vy =  sin(m_worldinfo.robots[ir].orient) * m_shoot_velocity;
			}
		}
		// release owner, clear shoot request and start moving
		m_ballinfo.owner.clear();
		m_shoot_velocity = 0.0;
		// also add a cooldown to prevent the ball being eaten again before it has escaped its owner
		// TODO: improve this solution as it is far from elegant and robust! It could even go wrong with negative int overflow :).
		m_cooldown = 15;
		stepAndPublish(dt);
		return;
	}

	// ball getting owned by a robot? check also that the ball is not being shot, escaping its previous owner
    TRACE("SimBall::update m_ballinfo.owner=%s  m_cooldown=%d", m_ballinfo.owner.c_str(), m_cooldown);
	if ((0 == m_ballinfo.owner.size()) && (m_cooldown-- <= 0))
	{
		TRACE("SimBall::update checking for new owner, ball_with_robot_radius=%f", m_config.ball_with_robot_radius);

        /* JFEI 2015-05-14 we cannot leave this to WorldModel. Simulator needs to decide
           how the ball moves. From both teams, several claims are coming which are not necessarily consistent.
           Also, subsequent calls on ballPossessionCallback will override each other...
           So the best way is to have simulator determine it, like it used to.
           */
        /*           
		if(m_BallPossessionType == WorldModelMsgs::BallPossession::TYPE_TEAMMEMBER)
		{
		    m_ballinfo.owner.clear();
		    m_ballinfo.owner.append("team");
		    m_ballinfo.owner.append(std::string(1, m_BallPossessionTeam));
		    m_ballinfo.owner.append("/robot");
		    m_ballinfo.owner.append(std::to_string(static_cast<long long>(m_BallPossessionRobotID)));
		    TRACE("SimBall::update getting pwnd by %s", m_ballinfo.owner.c_str());
		}
        */
		
		for (int ir=0; ir < (int)m_worldinfo.robots.size(); ++ir)
		{
			float ball_mouth_x = m_worldinfo.robots[ir].x + cos(m_worldinfo.robots[ir].orient) * m_config.ball_with_robot_radius * m_worldinfo.robots[ir].size/2;
			float ball_mouth_y = m_worldinfo.robots[ir].y + sin(m_worldinfo.robots[ir].orient) * m_config.ball_with_robot_radius * m_worldinfo.robots[ir].size/2;
			float dist2 = SQR(m_ballinfo.x - ball_mouth_x) + SQR(m_ballinfo.y - ball_mouth_y);
			TRACE("SimBall::update checking for new owner -- bot=%s, dist2=%f", m_worldinfo.robots[ir].name.c_str(), dist2);
			if (dist2 < SQR(m_config.ball_capture_radius))
			{
		        m_ballinfo.owner.clear();
		        m_ballinfo.owner.append(m_worldinfo.robots[ir].team);
		        m_ballinfo.owner.append("/");
		        m_ballinfo.owner.append(m_worldinfo.robots[ir].name);
		        TRACE("SimBall::update getting pwnd by %s", m_ballinfo.owner.c_str());
				break;
			}
		}
		
	}

	// if ball is owned by robot, then move along
	if (m_ballinfo.owner.size())
	{
		TRACE("SimBall::update moving along with owner %s", m_ballinfo.owner.c_str());
		for (size_t ir=0; ir < m_worldinfo.robots.size(); ++ir)
		{
			if (m_ballinfo.owner == m_worldinfo.robots[ir].team + "/" + m_worldinfo.robots[ir].name)
			{
				m_ballinfo.x = m_worldinfo.robots[ir].x + cos(m_worldinfo.robots[ir].orient) * m_config.ball_with_robot_radius * m_worldinfo.robots[ir].size/2;
				m_ballinfo.y = m_worldinfo.robots[ir].y + sin(m_worldinfo.robots[ir].orient) * m_config.ball_with_robot_radius * m_worldinfo.robots[ir].size/2;
				m_ballinfo.vx = m_worldinfo.robots[ir].vx; // TODO orient also influences speed
				m_ballinfo.vy = m_worldinfo.robots[ir].vy;
			}
		}
		stepAndPublish(dt);
		return;
	}

	TRACE("sorted collision handling");
	// because I have chosen to model goal walls with many small circles, we here need to sort obstacles, to resolve closest collision first
	// so: make one pile of obstacles. First resolve closest object. Then re-sort and iterate until no more collisions.

	// first build a list of obstacles, which is the same for each iteration
	// the sorting however will be different as the position changes each iteration
	// the static obstacles are computed once (m_obstacles, in constructor), to that list the dynamic obstacles are added
	std::vector<Obstacle> obstacles = m_obstacles;
	Vector2D ballpos(m_ballinfo.x, m_ballinfo.y);
	for (size_t ir = 0; (ir < m_worldinfo.robots.size())  && (m_cooldown <= 0); ++ir) // now the dynamic obstacles (i.e. robots), which are constant only during this time step
	{
		TRACE("adding robot obstacle %d", ir);
		Vector2D robotpos(m_worldinfo.robots[ir].x, m_worldinfo.robots[ir].y);
		Circle robotcircle(robotpos.x, robotpos.y, m_worldinfo.robots[ir].size/2);
		Vector2D robotspeed(m_worldinfo.robots[ir].vx, m_worldinfo.robots[ir].vy);
		TRACE("robotspeed (%6.2f, %6.2f)", robotspeed.x, robotspeed.y);
		// when a ball is entering a robots' mouth, it should not bounce on its virtual size. So we apply a trick: if the robot is facing the ball,
		// he gets a 10% radius so the ball will not bounce of him
		Vector2D ball_wrt_robot = ballpos - robotpos;
		Vector2D ball_wrt_mouth = ball_wrt_robot;
                ball_wrt_mouth.rotate(-m_worldinfo.robots[ir].orient);
		TRACE("ball_wrt_robot (%6.2f, %6.2f)", ball_wrt_robot.x, ball_wrt_robot.y);
		TRACE("ball_wrt_mouth (%6.2f, %6.2f)", ball_wrt_mouth.x, ball_wrt_mouth.y);
		// hardcoded 45degree
		if (fabs(ball_wrt_mouth.y) < ball_wrt_mouth.x)
		{
			TRACE("correcting radius based on facing angle");
			robotcircle.r *= 0.1;
		}
		obstacles.push_back(Obstacle(robotcircle, robotspeed));
	}
	TRACE("number of obstacles: %d", (int)obstacles.size());
	bool done = false;
	while (!done)
	{
		TRACE("collision loop");
		// re-sort the obstacles to find closest collision to resolve
		Vector2D my_pos(m_ballinfo.x, m_ballinfo.y);
		sort(obstacles.begin(), obstacles.end(), compare_distance_from_ball(my_pos, m_ballinfo.size/2));
	
		// only handle closest obstacle
		Obstacle closest_obstacle = obstacles[0];
		double closest_distance = vectorsize(closest_obstacle.posr.pos - my_pos) - closest_obstacle.posr.r - m_ballinfo.size/2;
		TRACE("closest_distance: %f", closest_distance);
		if (closest_distance < 0.00001)
		{
			// collision!
			resolveCollision(closest_obstacle.posr, closest_obstacle.speed, dt);
			obstacles.erase(obstacles.begin());
		}
		else
		{
			done = true;
		}
	}

	stepAndPublish(dt);
	// note: with this blind forward stepping it typically happens that the ball can enter the physical space of another object
	// this is resolved in resolveCollision, which calculates back the dt fraction that caused overlap, then apply it back after collision impulse change
	
} // void SimBall::update(double dt)

void SimBall::ballPossessionCallback(const BallPossessionSimConstPtr ballPossession)
{
    m_BallPossessionType = ballPossession->possessionType;
    m_BallPossessionRobotID = ballPossession->possessionRobotID;
    m_BallPossessionTeam = ballPossession->possessionTeam;    
	TRACE("Type=%d RobotID=%d Team=%c", m_BallPossessionType, m_BallPossessionRobotID, ballPossession->possessionTeam);
}

} // end of namespace simulator


int main(int argc, char **argv)
{
	ros::init(argc, argv, RobotsimNodeNames::ROBOTSIM_SIMBALL_NODENAME);
	ros::NodeHandlePtr nh_;
	nh_.reset(new ros::NodeHandle);

	/* Dynamic reconfiguration *//*
	dynamic_reconfigure::Server<simulator::simulatorConfig> srv;
	dynamic_reconfigure::Server<simulator::simulatorConfig>::CallbackType f;
	srv.setCallback(f);*/

	// construct and spin
	simulator::SimBall *ball = new simulator::SimBall(ros::NodeHandle(RobotsimNodeNames::ROBOTSIM_SIMBALL_NODENAME));
	ros::Rate updateRate(MOTION_FREQUENCY);
	while (ros::ok())
	{
		// make sure WorldInfo is present
		// so process 'visualize' always needs to be loaded first
		ros::spinOnce();
		ball->update(1.0 / MOTION_FREQUENCY);
		updateRate.sleep();
	}

	return 0;
}

