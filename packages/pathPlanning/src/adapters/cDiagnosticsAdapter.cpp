 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsAdapter.cpp
 *
 *  Created on: Sept 05, 2015
 *      Author: Erik Kouters
 */

#include "int/adapters/cDiagnosticsAdapter.hpp"
#include <cmath>
#include "rosMsgs/t_obstacle.h"
#include "rosMsgs/t_polygon2D.h"

cDiagnosticsAdapter::cDiagnosticsAdapter()
 : diagnostics::cDiagnosticsSender<rosMsgs::t_diag_pathpl>(diagnostics::DIAG_PATHPLANNING, 10.0, false)
{
    TRACE(">");
    TRACE("initialize msg");
    rosMsgs::t_diag_pathpl msg = get();
    msg.target.x = NAN;
    msg.target.y = NAN;
    msg.target.phi = NAN;
    msg.subtarget.x = NAN;
    msg.subtarget.y = NAN;
    msg.active = false;
    set(msg);
    TRACE("cDiagnosticsAdapter constructed OK");
    TRACE("<");
}    

void cDiagnosticsAdapter::setTarget(double x, double y, double phi)
{
    TRACE(">");
    rosMsgs::t_diag_pathpl msg = get();
    msg.target.x = x;
    msg.target.y = y;
    msg.target.phi = phi;
    set(msg);
    TRACE("<");
}

void cDiagnosticsAdapter::setSubTarget(double x, double y)
{
    TRACE(">");
    rosMsgs::t_diag_pathpl msg = get();
    msg.subtarget.x = x;
    msg.subtarget.y = y;
    set(msg);
    TRACE("<");
}

void cDiagnosticsAdapter::setObstacles(const std::vector<pp_obstacle_struct_t> obstacles, const std::vector<polygon2D>& areas, std::vector<linepoint2D>& projectedSpeedVectors)
{
    TRACE(">");
    rosMsgs::t_diag_pathpl msg = get();
    msg.obstacles.clear();
    msg.forbiddenAreas.clear();
    msg.speedArrowEndPoints.clear();

    int robotNum = getRobotNumber();
    int i = 100 * robotNum; // start with an idx of 100 * robotNum (r3 = 300) such that there is no collision with worldmodel's obstacles.
    std::vector<pp_obstacle_struct_t>::const_iterator it;
    for (it = obstacles.begin(); it != obstacles.end(); ++it)
    {
    	/*
    	 * Filter the non-worldmodel objects for visualization
    	 */
    	if(it->type == pp_obstacle_type::WORLDMODEL)
    	{
			rosMsgs::t_obstacle obst;
			obst.id = i;
			obst.x = it->location.x;
			obst.y = it->location.y;
			msg.obstacles.push_back(obst);
    	}
        i++;
    }

    for(auto itA = areas.begin(); itA != areas.end(); itA++)
    {
    	rosMsgs::t_polygon2D poly;
    	poly.id = i;
    	std::vector<Point2D> points = itA->getPoints();

    	for(auto itB = points.begin(); itB != points.end(); itB++)
    	{
    		rosMsgs::t_point2D point;

    		point.x = itB->x;
    		point.y = itB->y;

    		poly.points.push_back(point);
    	}
    	msg.forbiddenAreas.push_back(poly);
    	i++;
    }

    for(auto it = projectedSpeedVectors.begin(); it != projectedSpeedVectors.end(); it++)
    {
    	rosMsgs::t_linepoint2D linepoint;
    	linepoint.id = i;

    	linepoint.src.x = it->getSourcePoint2D().x;
    	linepoint.src.y = it->getSourcePoint2D().y;
    	linepoint.dst.x = it->getDestinationPoint2D().x;
    	linepoint.dst.y = it->getDestinationPoint2D().y;

    	msg.speedArrowEndPoints.push_back(linepoint);
    }

    set(msg);
    TRACE("<");
}

void cDiagnosticsAdapter::setActive(bool active)
{
    TRACE(">");
    rosMsgs::t_diag_pathpl msg = get();
    msg.active = active;
    set(msg);
    TRACE("<");
}


void cDiagnosticsAdapter::initializePlotData()
{
    TRACE(">");
    _hROS.reset(new ros::NodeHandle());
    _pPlotData = _hROS->advertise<rosMsgs::t_pp_plotdata>("g_pp_plotdata", 5, false);
    TRACE("<");
}

void cDiagnosticsAdapter::setPlotData(pp_plot_data_struct_t plotData)
{
    TRACE(">");
    rosMsgs::t_pp_plotdata msg;
    msg.target.x = plotData.pos_t.x;
    msg.target.y = plotData.pos_t.y;
    msg.target.phi = plotData.pos_t.phi;
    msg.target.vx = plotData.vel_t.x;
    msg.target.vy = plotData.vel_t.y;
    msg.target.vphi = plotData.vel_t.phi;
    msg.actual.x = plotData.pos_a.x;
    msg.actual.y = plotData.pos_a.y;
    msg.actual.phi = plotData.pos_a.phi;
    msg.actual.vx = plotData.vel_a.x;
    msg.actual.vy = plotData.vel_a.y;
    msg.actual.vphi = plotData.vel_a.phi;
    msg.x_pid_out = plotData.x_pid_out;
    msg.x_pid_p = plotData.x_pid_p;
    msg.x_pid_i = plotData.x_pid_i;
    msg.x_pid_d = plotData.x_pid_d;
    msg.y_pid_out = plotData.y_pid_out;
    msg.y_pid_p = plotData.y_pid_p;
    msg.y_pid_i = plotData.y_pid_i;
    msg.y_pid_d = plotData.y_pid_d;
    msg.phi_pid_out = plotData.phi_pid_out;
    msg.phi_pid_p = plotData.phi_pid_p;
    msg.phi_pid_i = plotData.phi_pid_i;
    msg.phi_pid_d = plotData.phi_pid_d;
    msg.tokyo_drift = plotData.tokyo_drift;

    _pPlotData.publish(msg);
    TRACE("<");
}
