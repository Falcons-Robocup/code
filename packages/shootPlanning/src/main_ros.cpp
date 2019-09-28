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
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <unistd.h>
#include <sys/types.h>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>


#include "int/cShootPlanner.hpp"
#include "int/cReconfigureAdapter.hpp"
#include "shootPlanning/ShootPlanningNodeConfig.h"
#include "ext/cShootPlanningNames.h"

#include "FalconsCommon.h"
#include "tracing.hpp"
#include "cDiagnostics.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

/* Globals */
cShootPlanner *shootPlanner = NULL;
cReconfigureAdapter _reconfigAdapter;

cRTDBInputAdapter _rtdbInputAdapter;
cRTDBOutputAdapter _rtdbOutputAdapter;

boost::mutex mtx;

using std::exception;
using std::cerr;
using std::endl;
using std::cout;

/* Configuration functionality */
void configureWorldModel()
{
    shootPlanning::ShootPlanningNodeConfig config;

    /* Call configuration file */
    _reconfigAdapter.reconfig_cb(config, 0);

}


void getYAMLEntries(const std::string& yamlFile, std::set<std::string>& yamlEntries)
{
    // Load the YAML and get all entries from "ShootPlanningNode".
    YAML::Node yamlNode = YAML::LoadFile(yamlFile);
    YAML::Node spNode = yamlNode[ShootPlanningNodeNames::shootplanning_nodename];

    YAML::const_iterator it;
    for(it = spNode.begin(); it != spNode.end(); ++it)
    {
        yamlEntries.insert( it->first.as<std::string>() );
    }
}

void getROSEntries(std::set<std::string>& rosEntries)
{
    std::map<std::string, double> rosEntriesMap;
    bool found = ros::param::get(ShootPlanningNodeNames::shootplanning_nodename, rosEntriesMap);
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
    ros::init(argc, argv, ShootPlanningNodeNames::shootplanning_nodename);
    //TRACE_INFO("shootPlanning starting"); // must be after ros::init
    ros::NodeHandle n; //manages an internal reference count to make starting and shutting down a node simple

    try
    {
        std::string configFileAbs(determineConfig("ShootPlanning"));

        _reconfigAdapter.initializeRC();

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
        
        // RTDB adapters
        _rtdbInputAdapter = cRTDBInputAdapter(shootPlanner);
        _rtdbOutputAdapter = cRTDBOutputAdapter();

        // spin
        _rtdbInputAdapter.waitForShootSetpoint();

        delete shootPlanner;
        shootPlanner = NULL;
    } 
    catch (exception &e)
    {
        cerr << "Error occurred:" << e.what() << endl;
        TRACE_ERROR("Error occurred: %s", e.what());
    }
}
