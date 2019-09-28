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
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <pwd.h> // for getpwuid()
#include <boost/thread.hpp>

#include "int/ballHandlingControl.hpp"
#include "ext/ballHandlingNames.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "int/adapters/cReconfigureAdapter.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

boost::thread _workerThreadWaitForBallHandlersFeedback;


void getYAMLEntries(const std::string& yamlFile, std::set<std::string>& yamlEntries)
{
    // Load the YAML and get all entries from "ballHandling".
    YAML::Node yamlNode = YAML::LoadFile(yamlFile);
    YAML::Node ppNode = yamlNode[ballHandlingNames::nodename];

    YAML::const_iterator it;
    for(it = ppNode.begin(); it != ppNode.end(); ++it)
    {
        yamlEntries.insert( it->first.as<std::string>() );
    }
}

void getROSEntries(std::set<std::string>& rosEntries)
{
    std::map<std::string, double> rosEntriesMap;
    bool found = ros::param::get(ballHandlingNames::nodename, rosEntriesMap);
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

void loadYAML(cReconfigureAdapter *reconfigAdapter)
{
    // Initialize reconfig adapter
    reconfigAdapter->initializeRC();

    // Determine YAMLs to load.
    std::string configFileAbs = determineConfig("BallHandling");
    std::string commonYaml = std::string(getpwuid(getuid())->pw_dir) + "/falcons/code/config/BallHandling.yaml";

    // All ROS initialization done.
    // Get YAML params.
    std::set<std::string> specificYamlEntries;
    getYAMLEntries(configFileAbs, specificYamlEntries);

    // Load common YAML
    std::set<std::string> commonYamlEntries;
    getYAMLEntries(commonYaml, commonYamlEntries);

    // Combine specific and common YAML
    specificYamlEntries.insert(commonYamlEntries.begin(), commonYamlEntries.end());

    // Get ROS params.
    std::set<std::string> rosEntries;
    getROSEntries(rosEntries);

    // Compare all ROS entries with the YAML entries
    if (rosEntries != specificYamlEntries)
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
        for (itSet = specificYamlEntries.begin(); itSet != specificYamlEntries.end(); ++itSet)
        {
            std::cout << *itSet << ", ";
        }
        std::cout << std::endl;

        throw std::runtime_error("YAML and ROS config not consistent! See stdout for details.");
    }

    // Performed parameter consistency check. Load specific YAML.
    std::string configFileCmd("rosparam load ");
    configFileCmd.append(configFileAbs);
    TRACE("system call: %s", configFileCmd.c_str());
    int retval = system(configFileCmd.c_str());
    TRACE("system call returned code %d", retval);

    // Performed parameter consistency check. Load common YAML.
    configFileCmd = std::string("rosparam load ");
    configFileCmd.append(commonYaml);
    TRACE("system call: %s", configFileCmd.c_str());
    retval = system(configFileCmd.c_str());
    TRACE("system call returned code %d", retval);

    // Trigger ReconfigureAdapter to reload all values from YAML and push into dynamic_reconfigure.
    reconfigAdapter->reloadParams();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ballHandlingNames::nodename);
    ros::NodeHandle n;

    INIT_TRACE;

    try
    {
        // setup adapters
        TRACE("setup adapters");
        cRTDBInputAdapter rtdbInputAdapter;
        cRTDBOutputAdapter rtdbOutputAdapter;
        TRACE("done setup adapters");

        // control algorithm
        TRACE("setup ballHandlingControl");
        ballHandlingControl controller(rtdbOutputAdapter);
        TRACE("done setup ballHandlingControl");

        rtdbInputAdapter = cRTDBInputAdapter(&controller);

        // Load YAML and do YAML<=>ROS consistency check
        cReconfigureAdapter *reconfigAdapter = new cReconfigureAdapter(&controller);
        loadYAML(reconfigAdapter);
        
        // hook up the services
        //cROSAdapterServiceProvider srvProvider(&motionPlanner);
  
        // wait for ballhandlers feedback in a separate thread
        _workerThreadWaitForBallHandlersFeedback = boost::thread(boost::bind(&cRTDBInputAdapter::waitForBallHandlersFeedback, &rtdbInputAdapter));

        // spin
        TRACE("spin");
        rtdbInputAdapter.waitForBallHandlersSetpoint();
        
        // cleanup
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    return 0;
}
