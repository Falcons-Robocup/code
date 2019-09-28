 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include <tracing.hpp>
#include <cmath>

cDiagnosticsAdapter::cDiagnosticsAdapter()
{
    TRACE(">");
    reset();
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
    TRACE("<");
}

void cDiagnosticsAdapter::reset()
{
    _target.reset();
    _subTarget.reset();
    _forbiddenAreas.clear();
}

void cDiagnosticsAdapter::send()
{
    if (_rtdb != NULL)
    {
        T_DIAG_PATHPLANNING msg;
        if (_subTarget)
        {
            wayPoint w;
            w.pos = _subTarget.get();
            w.vel = pose(); // velocity not used yet; future work
            msg.path.push_back(w);
        }
        if (_target)
        {
            wayPoint w;
            w.pos = _target.get();
            w.vel = pose(); // velocity not used yet; future work
            msg.path.push_back(w);
        }
        msg.forbiddenAreas = _forbiddenAreas;
        _rtdb->put(DIAG_PATHPLANNING, &msg);
        // reset for next iteration, to not repeat old targets etc.
        reset();
    }
}

void cDiagnosticsAdapter::setTarget(double x, double y, double phi)
{
    TRACE(">");
    pose p;
    p.x = x;
    p.y = y;
    p.Rz = phi;
    _target = p;
    TRACE("<");
}

void cDiagnosticsAdapter::setSubTarget(double x, double y)
{
    TRACE(">");
    pose p;
    p.x = x;
    p.y = y;
    p.Rz = 0;
    _subTarget = p;
    TRACE("<");
}

void cDiagnosticsAdapter::setObstacles(const std::vector<polygon2D>& areas, std::vector<linepoint2D>& projectedSpeedVectors)
{
    TRACE(">");
    int i = 100 * _myRobotId; // start with an idx of 100 * robotNum (r3 = 300) such that there is no collision with worldmodel's obstacles.

    for(auto itA = areas.begin(); itA != areas.end(); itA++)
    {
        polygon poly;
        poly.id = i;
        std::vector<Point2D> points = itA->getPoints();

        for(auto itB = points.begin(); itB != points.end(); itB++)
        {
            vec2d point;

            point.x = itB->x;
            point.y = itB->y;

            poly.points.push_back(point);
        }
        _forbiddenAreas.push_back(poly);
        i++;
    }

    for(auto it = projectedSpeedVectors.begin(); it != projectedSpeedVectors.end(); it++)
    {
        Vector2D src(it->getSourcePoint2D().x, it->getSourcePoint2D().y);
        Vector2D dst(it->getDestinationPoint2D().x, it->getDestinationPoint2D().y);
        Vector2D delta = dst - src;
        float h = delta.size();
        float w = 0.3;
        // only if distance between points is large enough, i.e. obstacle is moving fast enough
        if (h > 0.1)
        {
            // calculate polygon
            Vector2D perpendicular = delta;
            perpendicular.rotate(M_PI * 0.5);
            perpendicular.normalize(w);
            polygon poly;
            poly.id = i;
            Vector2D point = src + perpendicular * 0.5;
            poly.points.push_back(vec2d(point.x, point.y));
            point = src - perpendicular * 0.5;
            poly.points.push_back(vec2d(point.x, point.y));
            point = dst - perpendicular * 0.5;
            poly.points.push_back(vec2d(point.x, point.y));
            point = dst + perpendicular * 0.5;
            poly.points.push_back(vec2d(point.x, point.y));
            // store polygon
            _forbiddenAreas.push_back(poly);
            i++;
        }
    }
    TRACE("<");
}

void cDiagnosticsAdapter::initializePlotData()
{
    TRACE(">");
// TODO
/*
    _hROS.reset(new ros::NodeHandle());
    _pPlotData = _hROS->advertise<rosMsgs::t_pp_plotdata>("g_pp_plotdata", 5, false);
*/
    TRACE("<");
}

void cDiagnosticsAdapter::setPlotData(pp_plot_data_struct_t plotData)
{
    TRACE(">");
// TODO
/*
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
    msg.error_x = plotData.error_x;
    msg.error_y = plotData.error_y;
    msg.jerk_x = plotData.jerk_x;
    msg.jerk_y = plotData.jerk_y;
    msg.acc_x = plotData.acc_x;
    msg.acc_y = plotData.acc_y;
    msg.vel_x = plotData.vel_x;
    msg.vel_y = plotData.vel_y;
    msg.pos_x = plotData.pos_x;
    msg.pos_y = plotData.pos_y;

    _pPlotData.publish(msg);
*/
    TRACE("<");
}

