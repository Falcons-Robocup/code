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
 */

#ifndef ROBOTSIM_VISUALIZER_HPP
#define ROBOTSIM_VISUALIZER_HPP


#include <wx/wx.h>
#include <wx/event.h>
#include <wx/timer.h>

#include <ros/ros.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <simulator/Spawn.h>
#include <simulator/Kill.h>
#include <simulator/WorldInfo.h>
#include <simulator/BallInfo.h>
#include <simulator/simulatorConfig.h>
#include <simulator/Position.h>
#include "rosMsgs/t_sim_position.h"

#include <std_srvs/Empty.h>
#include <map>


namespace simulator
{

// auxiliary: single robot visualization	
class VisualizeRobot
{
public:
    VisualizeRobot(const ros::NodeHandle& nh, const wxImage& robot_image, float posx, float posy, float orient, std::string robotname, simulatorConfig *config);
    ~VisualizeRobot();

    void positionCallback(const rosMsgs::t_sim_position::ConstPtr& robot_pos);
    void update(double dt, float canvas_width, float canvas_height);
    void paint(wxDC& dc);

 // datamembers
    bool               m_active; // only when position is received
    float              m_pos_x;
    float              m_pos_y;
    float              m_old_pos_x;
    float              m_old_pos_y;
    float              m_orient;
    float              m_vel_x;
    float              m_vel_y;
    std::string        m_robotname;
    std::string        m_teamname;
    boost::posix_time::ptime _lastTimestamp;
        
private:
    ros::NodeHandle    m_nh;
    wxImage            m_robot_image;
    int                m_robotid;
    ros::Subscriber    m_position_sub;
    //ros::Subscriber    m_target_sub; // TODO visualize target
    ros::WallTime      m_last_command_time;
	simulatorConfig    *m_config;

}; // class VisualizeRobot


// auxiliary: ball visualization
class VisualizeBall
{
public:
	VisualizeBall(const ros::NodeHandle& nh, const wxImage& ball_image, simulatorConfig *config);
    ~VisualizeBall();

    void msgCallback(const BallInfoConstPtr& ball_info);

    void paint(wxDC& dc);

 // datamembers
    bool               m_active; // only when position is received
	BallInfo           m_ball_info;

private:
    ros::NodeHandle    m_nh;
    wxImage            m_ball_image;
    ros::Subscriber    m_msg_sub;
    simulatorConfig    *m_config;

}; // class VisualizeBall


// main visualizer, which draws the field, the robots and the ball
class Visualizer : public wxFrame
{

public:
    Visualizer(wxWindow* parent);
    ~Visualizer();
    void configCallback(simulator::simulatorConfig &config, uint32_t level);

private:
    void onUpdate(wxTimerEvent& evt);
    void onPaint(wxPaintEvent& evt);

    void updateRobots();
    void updateBall();
    void clear();

    bool spawnCallback(simulator::Spawn::Request&, simulator::Spawn::Response&);
    bool killCallback(simulator::Kill::Request&, simulator::Kill::Response&);

    // datamembers

    ros::NodeHandle   m_nh;
    wxTimer*          m_update_timer;
    wxMemoryDC        m_field_dc;
    wxBitmap          m_field_bitmap;
    uint64_t          m_frame_count;
    ros::WallTime     m_last_turtle_update;
    uint32_t          m_id_counter;
    wxImage           m_robot_image;
    wxImage           m_ball_image;
	simulatorConfig    m_config;
    float             m_canvas_width;
    float             m_canvas_height;
	WorldInfo         m_world_info_msg;
    VisualizeBall    *m_ball;
    typedef std::map<std::string, VisualizeRobot*> t_robots;
    ros::Publisher    m_worldinfo_pub;
    t_robots          m_robots;
    ros::ServiceServer m_spawn_srv;
    ros::ServiceServer m_kill_srv;
};

}

#endif

