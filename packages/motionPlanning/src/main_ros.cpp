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
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#include <boost/thread.hpp>

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>

#include "int/cMotionPlanner.hpp"
#include "ext/cMotionPlanningNames.hpp"

#include "int/cWorldModelInterface.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "int/adapters/configuration/cConfigMotionPlanningData.hpp"

#include "cDiagnostics.hpp"

void getYAMLEntries(const std::string& yamlFile, std::set<std::string>& yamlEntries)
{
    // Load the YAML and get all entries from "MotionPlanningNode".
    YAML::Node yamlNode = YAML::LoadFile(yamlFile);
    YAML::Node mpNode = yamlNode[MotionPlanningNodeNames::motionplanning_nodename];

    YAML::const_iterator it;
    for(it = mpNode.begin(); it != mpNode.end(); ++it)
    {
        yamlEntries.insert( it->first.as<std::string>() );
    }
}

void getROSEntries(std::set<std::string>& rosEntries)
{
    std::map<std::string, double> rosEntriesMap;
    bool found = ros::param::get(MotionPlanningNodeNames::motionplanning_nodename, rosEntriesMap);
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, MotionPlanningNodeNames::motionplanning_nodename);
    ros::NodeHandle n;

    INIT_TRACE;

    try
    {
    	std::string configFileAbs(determineConfig("motionPlanning"));
        // initialize all adapters

        cConfigMotionPlanningData *cfg = new cConfigMotionPlanningData();
        // Accuracy Obstacle_avoidance_enabled, Limiters_maxVelXY, Obstacle_Value_Min, Ball_Hue_Delta, Line_Hue_Center
	    // All ROS initialization done.
	    // Get YAML params.
	    std::set<std::string> yamlEntries;
	    getYAMLEntries(configFileAbs, yamlEntries);
	    // Get ROS params.
	    std::set<std::string> rosEntries;
	    getROSEntries(rosEntries);
        //alwaysLobshot
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
        
        cWorldModelInterface *wmInterface = new cWorldModelInterface();
        cRTDBOutputAdapter *rtdbOutput = new cRTDBOutputAdapter();
        cMotionPlanner motionPlanner(wmInterface, rtdbOutput);
        
        // input adapter triggers motionPlanner to start executing
        printf("Entering cRTDBInputAdapter\n");
        fflush(stdout);
        cRTDBInputAdapter *rtdbInput = new cRTDBInputAdapter(&motionPlanner);
        printf("Leaving cRTDBInputAdapter\n");
        fflush(stdout);

        // reconfigure adapter
        //cReconfigureAdapter _reconfigAdapter;
        
        // initialize ballhandlers ON
//        if (bhInterface != NULL)
//        {
//            motionPlanner.getBHI()->enableBallHandlers();
//        }
        
        // spin
        // TODO: execution architecture: wmInterface->update() needs to be poked each iteration
        //TRACE_INFO("motionPlanner initialized"); // containment for 2sec sleep in diagnosticsEvents... should use shared memory?

        rtdbInput->waitForActionData();
        
        // cleanup
        delete rtdbOutput;
        delete rtdbInput;
        delete wmInterface;
                
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    return 0;
}

