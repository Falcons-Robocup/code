 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: jfeitsma
 * Creation: 2014-04-06
 *
 * Redesigning simulator from Niels Koenraad.
 * Separate processes
 *   visualize
 *   simrobot N 
 *   simball
 * 
 * This is the visualization process.
 *
 * CHANGELOG
 * 2014-10-16 cut out wheel velocity and OROCOS layer, preparing for transition
 *            to firmware API from Prabu 
 */

#include "int/visualizer.hpp"
#include "FalconsCommon.h" // Tracer 
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <wx/bitmap.h> 

#define DEFAULT_BG_R 0x00//0x45
#define DEFAULT_BG_G 0x80//0x56
#define DEFAULT_BG_B 0x00//0xff



namespace simulator
{

// JFEI 2014-08-07 coordinate transformation decision wk1428: always play forward in y direction
// input as via coordinate system convention
// output for drawing :
//
// 0,0  -> x
// 
// y
// 
void transform(simulatorConfig *config, float x, float y, float orient, wxSize *img_size, float *draw_x, float *draw_y, float *draw_orient)
{
    TRACE("transform: x=%f y=%f orient=%f imgh=%d imgw=%d ppmx=%f ppmy=%f", x, y, orient, (int)img_size->GetHeight(), (int)img_size->GetWidth(), (float)config->pixels_per_meter_x, (float)config->pixels_per_meter_y);
    *draw_x = x;
    *draw_y = y;
    *draw_orient = orient - M_PI/2;
    if (!config->team1_left_to_right) {
        // rotate 180 degrees
        *draw_orient = *draw_orient + M_PI;
        *draw_x *= -1;
        *draw_y *= -1;
    }
    // translate field center to bitmap corner
    float tmp_draw_x = *draw_x + config->edge_x + config->field_size_x/2;
    float tmp_draw_y = *draw_y + config->edge_y + config->field_size_y/2;
    // mirror x,y and take PPM + draw image size into account
    TRACE("transform: tmp_draw_x=%f tmp_draw_y=%f draw_orient=%f", *draw_x, *draw_y, *draw_orient);
    *draw_y = tmp_draw_x * config->pixels_per_meter_x - (img_size->GetHeight() / 2);
    *draw_x = tmp_draw_y * config->pixels_per_meter_y - (img_size->GetWidth() / 2);
    TRACE("transform: draw_x=%f draw_y=%f draw_orient=%f", *draw_x, *draw_y, *draw_orient);
}

// first the implementations of class Visualizer
Visualizer::Visualizer(wxWindow* parent) :
		wxFrame(parent, wxID_ANY, wxT("ASML Turtle 5k - Simulator/visualizer"),
				wxDefaultPosition, wxSize(840, 600),
				wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER), m_frame_count(0)

{
	TRACE("Visualizer constructor start");

	srand(time(NULL));

	m_update_timer = new wxTimer(this);
	m_update_timer->Start(16);

	Connect(m_update_timer->GetId(), wxEVT_TIMER,
			wxTimerEventHandler(Visualizer::onUpdate), NULL, this);
	Connect(wxEVT_PAINT, wxPaintEventHandler(Visualizer::onPaint), NULL, this);

	m_nh.setParam("background_r", DEFAULT_BG_R);
	m_nh.setParam("background_g", DEFAULT_BG_G);
	m_nh.setParam("background_b", DEFAULT_BG_B);

	std::string images_path = ros::package::getPath("simulator") + "/images/";
	m_robot_image.LoadFile(
			wxString::FromAscii((images_path + "pacman_topright_20pixels.png").c_str()));
	m_robot_image.SetMask(true);
	m_robot_image.SetMaskColour(255, 255, 255);

	m_field_bitmap = wxBitmap(GetSize().GetWidth(), GetSize().GetHeight());
	m_field_dc.SelectObject(m_field_bitmap);
	clear();

	m_ball_image.LoadFile(
			wxString::FromAscii((images_path + "ball_9pixels.png").c_str()));
	m_ball_image.SetMask(true);
	m_ball_image.SetMaskColour(255, 255, 255);

	m_canvas_width = GetSize().GetWidth();
	m_canvas_height = GetSize().GetHeight();

	/* Dynamic reconfiguration */
	ros::NodeHandle tmp_nh("/simconfig");
	dynamic_reconfigure::Server<simulator::simulatorConfig> srv(tmp_nh);
	dynamic_reconfigure::Server<simulator::simulatorConfig>::CallbackType cb;
	cb = boost::bind(&Visualizer::configCallback, this, _1, _2);
	srv.setCallback(cb);

	// services to add/remove robot to the pool
	m_spawn_srv = m_nh.advertiseService("/simulator/spawn", &Visualizer::spawnCallback,
			this);
	m_kill_srv = m_nh.advertiseService("/simulator/kill", &Visualizer::spawnCallback,
		this);

	// world info publishing
	m_worldinfo_pub = m_nh.advertise<simulator::WorldInfo>(
			"/simulator/worldinfo", 10);

	m_ball = new VisualizeBall(m_nh, m_ball_image, &m_config);
	TRACE("Visualizer constructor end");
}


bool Visualizer::spawnCallback(simulator::Spawn::Request& req,
		simulator::Spawn::Response& res)
{
	TRACE("Visualizer spawnCallback start");
	ROS_INFO("Spawning robot %s", req.name.c_str());
	if (m_robots.count(req.name))
	{
		ROS_ERROR(
				"Tried to spawn turtle [%s], which already exist", req.name.c_str());
		return false;
	}
	m_robots[req.name] = new VisualizeRobot(m_nh, m_robot_image,
			m_canvas_width / 2, m_canvas_height / 2, 10, req.name, 
			&m_config);
	TRACE("Visualizer spawnCallback end OK");
	return true;
}

bool Visualizer::killCallback(simulator::Kill::Request& req,
		simulator::Kill::Response&)
{
	TRACE("Visualizer killCallback start");
	ROS_INFO("Killing robot %s", req.name.c_str());
	t_robots::iterator it = m_robots.find(req.name);
	if (it == m_robots.end())
	{
		ROS_ERROR(
				"Tried to kill turtle [%s], which does not exist", req.name.c_str());
		return false;
	}
	m_robots.erase(it);
	TRACE("Visualizer killCallback end OK");
	return true;
}

Visualizer::~Visualizer()
{
	delete m_update_timer;
}

void Visualizer::clear()
{
	int r = DEFAULT_BG_R;
	int g = DEFAULT_BG_G;
	int b = DEFAULT_BG_B;

	m_nh.param("background_r", r, r);
	m_nh.param("background_g", g, g);
	m_nh.param("background_b", b, b);
	std::string images_path = ros::package::getPath("simulator") + "/images/";
	m_field_dc.SetBackground(
			wxBitmap(
					wxImage(
							wxString::FromAscii(
									(images_path + "robocup_veld2.png").c_str()))));
	m_field_dc.Clear();

}

void Visualizer::onUpdate(wxTimerEvent& evt)
{
	ros::spinOnce();
	updateRobots();

	if (!ros::ok())
	{
		Close();
	}
}

void Visualizer::onPaint(wxPaintEvent& evt)
{
	wxPaintDC dc(this);
	dc.DrawBitmap(m_field_bitmap, 0, 0, true);

	t_robots::iterator it = m_robots.begin();
	t_robots::iterator end = m_robots.end();
	for (; it != end; ++it)
	{
		it->second->paint(dc);
	}
	m_ball->paint(dc);
}

void Visualizer::updateRobots()
{

	TRACEF("Visualizer updateRobots");
	if (m_last_turtle_update.isZero())
	{
		m_last_turtle_update = ros::WallTime::now();
		return;
	}

	TRACE("Visualizer updating %d robots", m_robots.size());
	t_robots::iterator it = m_robots.begin();
	t_robots::iterator end = m_robots.end();
	for (; it != end; ++it)
	{
		it->second->update(0.016, m_canvas_width,
				m_canvas_height);
	}
	++m_frame_count;

	// repaint
	if (m_frame_count % 3 == 0)
	{
		Refresh();
	}

	// write all locations etc to simulator WorldInfo topic (to make individual agents able to calculate/prevent collisions etc)
	m_world_info_msg.ball = m_ball->m_ball_info;
	m_world_info_msg.edge_x = m_config.edge_x;
	m_world_info_msg.edge_y = m_config.edge_y;
	m_world_info_msg.goalsize = m_config.goal_size;
	m_world_info_msg.xsize = m_config.field_size_x; // m_canvas_width / m_config.pixels_per_meter_x;
	m_world_info_msg.ysize = m_config.field_size_y; // m_canvas_height / m_config.pixels_per_meter_y;
	m_world_info_msg.robots.resize(m_robots.size());
	it = m_robots.begin();
	int i = 0;
	for (; it != end; ++it)
	{
		m_world_info_msg.robots[i].name = it->second->m_robotname;
		m_world_info_msg.robots[i].team = it->second->m_teamname;
		m_world_info_msg.robots[i].x = it->second->m_pos_x;
		m_world_info_msg.robots[i].y = it->second->m_pos_y;
		m_world_info_msg.robots[i].orient = it->second->m_orient;
		m_world_info_msg.robots[i].vx = it->second->m_vel_x;
		m_world_info_msg.robots[i].vy = it->second->m_vel_y;
		m_world_info_msg.robots[i].size = m_config.robot_size;
		TRACE("Visualizer robotsize=%f", m_config.robot_size);
		i++;
	}
	m_worldinfo_pub.publish(m_world_info_msg);
	TRACE("Visualizer published worldinfo");
}

// implementations of class VisualizeRobot

VisualizeRobot::VisualizeRobot(const ros::NodeHandle& nh,
		const wxImage& robot_image, float posx, float posy, float orient,
		std::string robotname, simulatorConfig *config) :
		m_active(false), m_pos_x(posx), m_pos_y(posy), m_orient(orient), 
		m_old_pos_x(0), m_old_pos_y(0),
		m_nh(nh),
 		m_robot_image(robot_image), m_config(config)
{
	TRACE("VisualizeRobot::constructor");
    _lastTimestamp = boost::posix_time::microsec_clock::local_time();
	// support both running from falcons_control.py and 
	// [soon to be obsolete] simulator_control.py
	// falcons_control will call with a name like "teamA/robot1"
	// which we use in the namespace but then remove team
	// to not influence remainder of visualizer
	m_position_sub = m_nh.subscribe(
		"/" + robotname + "/g_sim_position", 100,
		&VisualizeRobot::positionCallback, this);
	TRACE("VisualizeRobot::constructor namespace=%s", robotname.c_str());
	m_teamname = "none";
	if (robotname.find("team") != std::string::npos)
	{
        m_teamname = robotname.substr(0,5);
		robotname = robotname.substr(6);
	}
	m_robotname = robotname;
	TRACE("VisualizeRobot::constructor teamname=%s robotname=%s", m_teamname.c_str(), m_robotname.c_str());
}

void VisualizeRobot::paint(wxDC& dc)
{
	if (m_active)
	{
		wxSize robot_size = wxSize(m_robot_image.GetWidth(), m_robot_image.GetHeight());
                float draw_x = 0.0;
                float draw_y = 0.0;
                float draw_orient = 0.0;
                transform(m_config, m_pos_x, m_pos_y, m_orient, &robot_size, &draw_x, &draw_y, &draw_orient);
		// first rotate robot image
		wxImage rotated_image = m_robot_image.Rotate(draw_orient - M_PI/4.0, wxPoint(m_robot_image.GetWidth() / 2, m_robot_image.GetHeight() / 2), false);
		TRACE("VisualizeRobot::paint imgsize = %d %d", rotated_image.GetHeight(), rotated_image.GetWidth());

		// prepare bitmap
		wxBitmap rotated_bmp = wxBitmap(rotated_image);
                // JFEI workaround... I dont get it - where does this offset come from?!
                // I had to send the robot to 0,0 and play the ball at it, to figure out the hardcoded offset...
                draw_x -= robot_size.GetWidth() * 0.25;
                draw_y -= robot_size.GetHeight() * 0.25;
                // round and draw
                draw_x = round(draw_x);
                draw_y = round(draw_y);
		TRACE("VisualizeRobot::paint %s debug PPM: x=%f y=%f", m_robotname.c_str(), m_config->pixels_per_meter_x, m_config->pixels_per_meter_y);
		dc.DrawBitmap(rotated_bmp, draw_x, draw_y, true);
		TRACE("VisualizeRobot::paint %s: x=%f y=%f", m_robotname.c_str(), draw_x, draw_y);
	}
	else
	{
		TRACE("VisualizeRobot::paint NOT YET ACTIVE");
	}
}

void VisualizeRobot::update(double dt, float canvas_width, float canvas_height)
{
	TRACE("VisualizeRobot::update %s: x=%f y=%f", m_robotname.c_str(), m_pos_x,m_pos_y);

}

void VisualizeRobot::positionCallback(const rosMsgs::t_sim_position::ConstPtr& robot_pos)
{
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - _lastTimestamp;
    double dt = (double)diff.total_microseconds() * 1e-6;
	TRACE("VisualizeRobot::positionCallback: oldx=%6.2f oldy=%6.2f TST=%d dt=%6.2f", m_old_pos_x, m_old_pos_y, diff.total_microseconds(), dt);
	    
	m_pos_x = (float) robot_pos->x;
	m_pos_y = (float) robot_pos->y;
	m_vel_x = (float) (m_pos_x - m_old_pos_x) / dt;
	m_vel_y = (float) (m_pos_y - m_old_pos_y) / dt;
	m_orient = (float) robot_pos->orient;
	m_active = true;
	TRACE("VisualizeRobot::positionCallback %s: x=%6.2f y=%6.2f orient=%f vx=%6.2f vy=%6.2f", m_robotname.c_str(), m_pos_x, m_pos_y, m_orient, m_vel_x, m_vel_y);
	// prepare for next call
    _lastTimestamp = boost::posix_time::microsec_clock::local_time();
    m_old_pos_x = m_pos_x;
    m_old_pos_y = m_pos_y;
}


// implementations of class VisualizeBall

VisualizeBall::VisualizeBall(const ros::NodeHandle& nh,
		const wxImage& ball_image, simulatorConfig *config) :
		m_active(false), m_nh(nh), m_config(config)
{
	m_ball_image = ball_image;
	m_msg_sub = m_nh.subscribe(
			"/simulator/ball", 100,
			&VisualizeBall::msgCallback, this);
	TRACE("VisualizeBall constructed");
}

void VisualizeBall::msgCallback(const BallInfoConstPtr& ball_info)
{
	m_ball_info = *ball_info;
	m_active = true;
	TRACE("VisualizeBall::msgCallback x=%f y=%f owner=%s", m_ball_info.x, m_ball_info.y, m_ball_info.owner.c_str());
}


void VisualizeBall::paint(wxDC& dc)
{
	if (m_active)
	{
//		wxImage image = m_ball_image.Rotate(0, wxPoint(m_ball_image.GetWidth() / 2, m_ball_image.GetHeight() / 2), false);
		wxImage image = m_ball_image;
		TRACE("VisualizeBall::paint imgsize = %d %d", image.GetHeight(), image.GetWidth());

		// draw bitmap
		wxBitmap bmp = wxBitmap(image);
		wxSize bmp_size = wxSize(bmp.GetWidth(), bmp.GetHeight());
                float draw_x = 0.0;
                float draw_y = 0.0;
                float draw_orient = 0.0; // unused
                transform(m_config, m_ball_info.x, m_ball_info.y, 0, &bmp_size, &draw_x, &draw_y, &draw_orient);
		TRACE("VisualizeBall::paint x=%f y=%f", draw_x, draw_y);
		dc.DrawBitmap(bmp, draw_x, draw_y, true);
	}
	else
	{
		TRACE("VisualizeBall::paint NOT YET ACTIVE");
	}
}



void Visualizer::configCallback(simulator::simulatorConfig &config, uint32_t level)
{
    m_config = config;
	TRACE("Visualizer::configCallback x=%f y=%f", m_config.pixels_per_meter_x, m_config.pixels_per_meter_y);

} // end configCallback()


} // end of namespace

