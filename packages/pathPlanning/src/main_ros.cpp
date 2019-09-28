 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main_ros.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */


#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>

#include <ros/ros.h>

#include "ext/cPathPlanningNames.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "int/adapters/cDiagnosticsAdapter.hpp"
#include "int/adapters/ConfigAdapter.hpp"
#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp"

#include "int/cPathPlanningData.hpp"
#include "int/cPathPlanningMain.hpp"
#include "int/cAbstractPathPlanning.hpp"

#include "int/facilities/cObstacleFacilities.hpp"

#define Y_POS_KEEPER_AREA 13.0
#define X_POS_KEEPER_AREA 10.0

using std::cout;
using std::endl;

cPathPlanningData _ppData;
cPathPlanningMain _ppMain;

cRTDBInputAdapter _rtdbInputAdapter;
cRTDBOutputAdapter _rtdbOutputAdapter;

ConfigAdapter *_configAdapter;


boost::thread _workerThread;


void iteratePP()
{
    TRACE_FUNCTION("");
    TRACE(">");
            
    // Convert field coordinates target to robot coordinates
    Position2D target, curPos;
    _ppData.getTarget(target);
    _ppData.getPosition(curPos);

    Position2D deltaPosition = target.transform_fcs2rcs(curPos);
    _ppData.setDeltaPosition(deltaPosition);

    _workerThread.interrupt();

    TRACE("<");
}

/*!
 * Used to publish speed from ppData to ROS
 */
void publishSpeed(const Velocity2D& vel_rcs)
{
    Velocity2D tmp = Velocity2D(vel_rcs);
    TRACE("> %s", tmp.tostr());

    bool robotStop;
    _ppData.getRobotStop(robotStop);

    T_ROBOT_VELOCITY_SETPOINT targetVel;
    if (robotStop)
    {
        targetVel.x = 0.0;
        targetVel.y = 0.0;
        targetVel.Rz = 0.0;
    }
    else
    {
        targetVel.x = vel_rcs.x;
        targetVel.y = vel_rcs.y;
        targetVel.Rz = vel_rcs.phi;
    }

    _rtdbOutputAdapter.setRobotVelocitySetpoint(targetVel);

    TRACE("<");
}

/*!
 * Used by the PathPlanning algorithm to set the subtarget for diagnostics purposes over ROS.
 */
void publishSubtarget(const double& x, const double& y)
{
    TRACE(">");
    cDiagnosticsAdapter::getInstance().setSubTarget(x, y);
    TRACE("<");
}

/*!
 * Used by the PathPlanning algorithm to set the subtarget for diagnostics purposes over ROS.
 */
void publishObstacles(const std::vector<polygon2D>& areas, std::vector<linepoint2D>& projectedSpeedVectors)
{
    TRACE(">");
    cDiagnosticsAdapter::getInstance().setObstacles(areas, projectedSpeedVectors);
    TRACE("<");
}
/*!
 * Used by the PathPlanning algorithm to publish diagnostic plotting data over ROS.
 */
void publishPlotData(const pp_plot_data_struct_t& plotData)
{
    TRACE(">");
    cDiagnosticsAdapter::getInstance().setPlotData(plotData);
    TRACE("<");
}

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    INIT_TRACE;

    // Determine YAML to load.
    std::string configFileAbs = determineConfig("PathPlanning");

    ros::init(argc, argv, PathPlanningNodeNames::pathplanning_nodename);

    // Bind publishSpeed function to ppData
    publishSpeedFunctionType publishSpeedFunc;
    publishSpeedFunc = boost::bind(&publishSpeed, _1);
    _ppData.publishSpeed = publishSpeedFunc;

    // Bind publishSubtarget function to ppData
    publishSubtargetFunctionType publishSubtargetFunc;
    publishSubtargetFunc = boost::bind(&publishSubtarget, _1, _2);
    _ppData.publishSubtarget = publishSubtargetFunc;

    // Bind publishObstacles function to ppData
    publishObstaclesFunctionType publishObstaclesFunc;
    publishObstaclesFunc = boost::bind(&publishObstacles, _1, _2);
    _ppData.publishObstacles = publishObstaclesFunc;

    // Bind publishPlotData function to ppData
    publishPlotDataFunctionType publishPlotDataFunc;
    publishPlotDataFunc = boost::bind(&publishPlotData, _1);
    _ppData.publishPlotData = publishPlotDataFunc;

    // Bind iterate function to all adapters
    iterateFunctionType iterateFunc;
    iterateFunc = boost::bind(&iteratePP);

    _rtdbInputAdapter = cRTDBInputAdapter(_ppData, iterateFunc);
    _rtdbOutputAdapter = cRTDBOutputAdapter();
    
    // Setup ConfigAdapter
    _configAdapter = new ConfigAdapter(_ppData);
    _configAdapter->loadYAML(configFileAbs);

    cDiagnosticsAdapter::getInstance().initializePlotData();

    /* Cold start: Set target to self. */
    Position2D pos;
    _ppData.getPosition(pos);
    _ppData.setTarget(pos);

    _ppMain._ppData = &_ppData;

    // Create static forbidden areas for goal avoidance

    poiInfo ownGoalAreaCornerLeft;
    poiInfo ownGoalAreaCornerRight;
    poiInfo oppGoalAreaCornerLeft;
    poiInfo oppGoalAreaCornerRight;
    //poiInfo ownCornerLeft;
    //poiInfo ownCornerRight;
    //poiInfo oppCornerLeft;
    //poiInfo oppCornerRight;
    std::vector<polygon2D> areas;

    cEnvironmentField::getInstance().getFieldPOI(poiName::P_OWN_GOALAREA_CORNER_LEFT, ownGoalAreaCornerLeft);
    cEnvironmentField::getInstance().getFieldPOI(poiName::P_OWN_GOALAREA_CORNER_RIGHT, ownGoalAreaCornerRight);
    cEnvironmentField::getInstance().getFieldPOI(poiName::P_OPP_GOALAREA_CORNER_LEFT, oppGoalAreaCornerLeft);
    cEnvironmentField::getInstance().getFieldPOI(poiName::P_OPP_GOALAREA_CORNER_RIGHT, oppGoalAreaCornerRight);
    //cEnvironmentField::getInstance().getFieldPOI(poiName::P_OWN_CORNER_LEFT, ownCornerLeft);
    //cEnvironmentField::getInstance().getFieldPOI(poiName::P_OWN_CORNER_RIGHT, ownCornerRight);
    //cEnvironmentField::getInstance().getFieldPOI(poiName::P_OPP_CORNER_LEFT, oppCornerLeft);
    //cEnvironmentField::getInstance().getFieldPOI(poiName::P_OPP_CORNER_RIGHT, oppCornerRight);


    Area2D ownGoal = Area2D(ownGoalAreaCornerLeft.x, -Y_POS_KEEPER_AREA, ownGoalAreaCornerRight.x, ownGoalAreaCornerLeft.y);
    Area2D oppGoal = Area2D(oppGoalAreaCornerLeft.x, oppGoalAreaCornerRight.y, oppGoalAreaCornerRight.x, Y_POS_KEEPER_AREA);

    //Area2D leftBorder = Area2D(ownCornerLeft.x - 0.75, -Y_POS_KEEPER_AREA, ownCornerRight.x + 0.75, ownCornerLeft.y - 0.75);
    //Area2D rightBorder = Area2D(oppCornerLeft.x - 0.75, oppCornerLeft.y + 0.75, oppCornerRight.x + 0.75, Y_POS_KEEPER_AREA);
    //Area2D topBorder = Area2D(-X_POS_KEEPER_AREA, ownCornerLeft.y - 1.0, ownCornerLeft.x - 0.75, oppCornerRight.y + 1.0);
    //Area2D bottomBorder = Area2D(ownCornerRight.x + 0.75, ownCornerLeft.y - 1.0, X_POS_KEEPER_AREA, oppCornerRight.y + 1.0);

    areas.push_back(ownGoal);
    areas.push_back(oppGoal);
    //areas.push_back(leftBorder);
    //areas.push_back(rightBorder);
    //areas.push_back(topBorder);
    //areas.push_back(bottomBorder);
    //_ppData.setStaticForbiddenAreas(areas); // this is controlled by teamplay, taking into account robot role and (if applicable) rules


    _workerThread = boost::thread(boost::bind(&cPathPlanningMain::iterate, &_ppMain));

    signal(SIGINT, mySigintHandler);

    _rtdbInputAdapter.waitForMotionSetpoint();

    TRACE("Spinner stopped, waiting for thread.");
    _workerThread.join();
    TRACE("Done. Terminating.");
    return 0;
}

