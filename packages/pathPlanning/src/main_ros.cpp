 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include "ext/cPathPlanningNames.hpp"

#include "int/adapters/cWorldModelAdapter.hpp"
#include "int/adapters/cTeamplayAdapter.hpp"
#include "int/adapters/cSetSpeedAdapter.hpp"
#include "int/adapters/cDiagnosticsAdapter.hpp"
#include "int/adapters/cReconfigureAdapter.hpp"
#include "cDiagnosticsEvents.hpp"
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

cWorldModelAdapter _wmAdapter;
cTeamplayAdapter _tpAdapter;
cSetSpeedAdapter _setSpeedAdapter;
cReconfigureAdapter _reconfigAdapter;


boost::thread _workerThread;

void getYAMLEntries(const std::string& yamlFile, std::set<std::string>& yamlEntries)
{
    // Load the YAML and get all entries from "PathPlanningNode".
    YAML::Node yamlNode = YAML::LoadFile(yamlFile);
    YAML::Node ppNode = yamlNode[PathPlanningNodeNames::pathplanning_nodename];

    YAML::const_iterator it;
    for(it = ppNode.begin(); it != ppNode.end(); ++it)
    {
        yamlEntries.insert( it->first.as<std::string>() );
    }
}

void getROSEntries(std::set<std::string>& rosEntries)
{
    std::map<std::string, double> rosEntriesMap;
    bool found = ros::param::get(PathPlanningNodeNames::pathplanning_nodename, rosEntriesMap);
    if (!found)
    {
        throw std::runtime_error("Unable to load ROS parameters.");
    }

    // Add ROS params to std::set
    std::map<std::string, double>::const_iterator it;
    for (it = rosEntriesMap.begin(); it != rosEntriesMap.end(); ++it)
    {
        rosEntries.insert(it->first);
    }
}

void iteratePP()
{
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
    _setSpeedAdapter.setSpeed(vel_rcs);
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
void publishObstacles(const std::vector<pp_obstacle_struct_t>& obstacles, const std::vector<polygon2D>& areas, std::vector<linepoint2D>& projectedSpeedVectors)
{
    TRACE(">");
    cDiagnosticsAdapter::getInstance().setObstacles(obstacles, areas, projectedSpeedVectors);
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

int main(int argc, char **argv)
{
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
    publishObstaclesFunc = boost::bind(&publishObstacles, _1, _2, _3);
    _ppData.publishObstacles = publishObstaclesFunc;

    // Bind publishPlotData function to ppData
    publishPlotDataFunctionType publishPlotDataFunc;
    publishPlotDataFunc = boost::bind(&publishPlotData, _1);
    _ppData.publishPlotData = publishPlotDataFunc;

    // Bind iterate function to all adapters
    iterateFunctionType iterateFunc;
    iterateFunc = boost::bind(&iteratePP);

    _wmAdapter = cWorldModelAdapter(_ppData, iterateFunc);
    _tpAdapter = cTeamplayAdapter(_ppData, iterateFunc);
    _setSpeedAdapter = cSetSpeedAdapter(_ppData, iterateFunc);
    _reconfigAdapter = cReconfigureAdapter(_ppData, iterateFunc);

    _wmAdapter.initializeWM();
    _tpAdapter.initializeTP();
    _setSpeedAdapter.initializeSetSpeed();
    _reconfigAdapter.initializeRC();
    cDiagnosticsAdapter::getInstance().initializePlotData();

    // All ROS initialization done.
    // Get YAML params.
    std::set<std::string> yamlEntries;
    getYAMLEntries(configFileAbs, yamlEntries);

    // Get ROS params.
    std::set<std::string> rosEntries;
    getROSEntries(rosEntries);

    // Compare all ROS entries with the YAML entries
    if (rosEntries != yamlEntries)
    {
        // Consistency check between YAML entries and ROS entries failed.
        // Print both sets.
        std::cout << "YAML and ROS config not consistent!" << std::endl;

        std::cout << "ROS entries: " << std::endl;
        std::set<std::string>::const_iterator itSet;
        for (itSet = rosEntries.begin(); itSet != rosEntries.end(); ++itSet)
        {
            std::cout << *itSet << ", ";
        }
        std::cout << std::endl;

        std::cout << "YAML entries: " << std::endl;
        for (itSet = yamlEntries.begin(); itSet != yamlEntries.end(); ++itSet)
        {
            std::cout << *itSet << ", ";
        }
        std::cout << std::endl;

        throw std::runtime_error("YAML and ROS config not consistent! See stdout for details.");
    }

    // Performed parameter consistency check. Load YAML.
    std::string configFileCmd("rosparam load ");
    configFileCmd.append(configFileAbs);
    TRACE("system call: %s", configFileCmd.c_str());
    int retval = system(configFileCmd.c_str());
    TRACE("system call returned code %d", retval);

    // Trigger ReconfigureAdapter to reload all values from YAML and push into dynamic_reconfigure.
    _reconfigAdapter.reloadParams();

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
    _ppData.setStaticForbiddenAreas(areas);


    _workerThread = boost::thread(boost::bind(&cPathPlanningMain::iterate, &_ppMain));

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin(); // spin() will not return until the node has been shutdown
}

