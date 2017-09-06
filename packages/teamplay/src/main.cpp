 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main.cpp
 *
 *  Created on: Oct 13, 2015
 *      Author: Jan Feitsma
 */

#define TEAMPLAY_NODENAME ("teamplay_main") 

/* System includes */
#include <stdexcept>
#include "ros/ros.h"

/* Other packages includes */
#include "rosMsgs/t_worldmodel_calculated.h"
#include "cDiagnosticsEvents.hpp"
#include "cDiagnosticsDutyCycle.hpp"
#include "tracer.hpp"

/* Teamplay includes: input adapters */
#include "int/adapters/configuration/cConfigInterceptBall.hpp"
#include "int/adapters/configuration/cConfigRules.hpp"
#include "int/adapters/configuration/cConfigShooting.hpp"
#include "int/adapters/configuration/cConfigStrategy.hpp"
#include "int/cRosAdapterWorldModel.hpp"
#include "ext/cUdpAdapterRefboxSignals.hpp"
#include "int/adapters/cControlInterfaceROS.hpp"

/* Teamplay includes: functionality */
#include "int/cWorldStateFunctions.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/cDecisionTree.hpp"
#include "int/gameStateManager.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/ownRobotStore.hpp"
#include "int/stores/teamMatesStore.hpp"
#include "int/types/intentionSync.hpp"
#include "int/utilities/trace.hpp"

namespace {
// The memory stack.
std::map<robotNumber,memoryStackType> robotMemoryStack;
std::map<robotNumber,memoryStackNodes> robotNodes;
int memoryStackIdx = -1; // Nothing on the stack. If an element is added, it is raised to 0, which points to the first element.

void trace_error (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        diagnostics::diag_error(event);
    }

void trace_info (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        diagnostics::diag_info(event);
    }

void trace (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        trace_write_event(event);
    }
    
    
#define EXPECTED_FREQUENCY (30.0) // TODO store somewhere common, provide by heartBeat? simulator uses 10Hz, real robot 30Hz
#define WARNING_THRESHOLD (0.4)
diagnostics::cDiagnosticsDutyCycle dutyCycleObserver("teamplay", EXPECTED_FREQUENCY, WARNING_THRESHOLD);

} // unnamed namespace


int main(int argc, char **argv)
{
    /* Install the Falcons specific trace handlers */
    teamplay::traceRedirect::getInstance().setTraceFunction(&trace);
    teamplay::traceRedirect::getInstance().setTraceErrorFunction(&trace_error);
    teamplay::traceRedirect::getInstance().setTraceInfoFunction(&trace_info);

	try
	{
		// ROS initialization
		ros::init(argc, argv, TEAMPLAY_NODENAME);
		ros::Time::init();
    	//TRACE_INFO("teamplay starting"); // must be after ros::init

		// setup the refbox signal listener by creating the object once (singleton)
		// (note that this is done via callback, so no periodical update in the loop below)
		cUdpAdapterRefboxSignals::getInstance();

		// Create sockets for the intention syncing
		intentionSync::getInstance();

		// setup the teamplay interface (e.g. for robot control)
		cControlInterfaceROS controlInterfaceROS;

		// Load the (dynamically reconfigurable) configuration parameters
        cConfigInterceptBall::getInstance();
		cConfigRules::getInstance();
		cConfigShooting::getInstance();
		cConfigStrategy::getInstance();

		// Load decision trees into memory
		cDecisionTree::getInstance();

		// Initialize the gamestate (to neutral stopped)
		teamplay::gameStateStore::getInstance();

        // Initialize ROS adapter including subscription on worldmodel heartbeat  (the heartbeat will call the  main loop function
        cRosAdapterWorldModel::getInstance();

		/* Loop forever */
		ros::spin();
	}
	catch (std::exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
	}
	catch (...)
    {
        TRACE_ERROR("Caught unknown exception!");
    }
}


void cb_worldModelUpdated(const worldModel::t_wmInfo::ConstPtr& msg)
{
	try
	{
        dutyCycleObserver.pokeStart();
        
	    cRosAdapterWorldModel::getInstance().update(msg); // call update functions to retrieve latest worldmodel datafeed

        // Refresh gamestate: check whether a transition from "setpiece execute" to neutral playing is allowed
        teamplay::gameStateManager::getInstance().refreshGameState();

		treeEnum gameState = teamplay::gameStateStore::getInstance().getGameState_treeEnum();

		// Get override state
		cOverrideState overrideState;
		cTeamplayControlInterface::getInstance().getOverrideState(overrideState);

		// Stimulate gamestate?
		if (overrideState.active && (overrideState.level == overrideLevelEnum::GAMESTATE))
		{
			TRACE_ERROR("trying to override gamestate -- NOT IMPLEMENTED");
		}

		robotNumber myRobotNr = getRobotNumber();

		// Make sure my robot number exists in the memory stack (before dereferencing)
		if (robotMemoryStack.find(myRobotNr) == robotMemoryStack.end())
		{
		    // Add memory of this robot
		    robotMemoryStack.insert( std::make_pair(myRobotNr, memoryStackType()) );
		}
		if (robotNodes.find(myRobotNr) == robotNodes.end())
		{
		    // Add nodes of this robot
		    robotNodes.insert( std::make_pair(myRobotNr, memoryStackNodes()) );
		}

		// Determine the role for my teammembers.
		// My decisions depend on whether some teammembers are present. (e.g., attacker assist)

		// TODO - should we clear this memory before recomputing? Or at the end of an iteration? If a robot drops out, his role will remain in memory.
		// However, all functions that use "getRobotRole" iterate on active_robots, so that should always be fine.

        // Stash own robot, new style:
        teamplay::teamMatesStore::getTeamMatesIncludingGoalie().add(teamplay::ownRobotStore::getOwnRobot());
        teamplay::ownRobotStore::getInstance().stashOwnRobot();

        // Old style:
        std::vector<robotNumber> active_robots;
        cWorldModelInterface::getInstance().getActiveRobots(active_robots);
        std::vector<robotNumber>::const_iterator it;
        for (it = active_robots.begin(); it != active_robots.end(); ++it)
        {
            if (*it != myRobotNr)
            {
                std::map<std::string, std::string> params;

                if (robotMemoryStack.find(*it) == robotMemoryStack.end())
                {
                    // Add memory of this robot
                    robotMemoryStack.insert( std::make_pair(*it, memoryStackType()) );
                }
                if (robotNodes.find(*it) == robotNodes.end())
                {
                    // Add nodes of this robot
                    robotNodes.insert( std::make_pair(*it, memoryStackNodes()) );
                }

                // Set robot ID at WSF so that any questions asked towards WSF are as if it was this robot asking it, old style:
                cWorldStateFunctions::getInstance().setRobotID(*it);

                // New style:
                teamplay::ownRobotStore::getInstance().replaceOwnRobot(teamplay::robot(*it));

                // Stash the "new own robot" in the list of teammates
                teamplay::teamMatesStore::getInstance().stashRobot(*it);

                // Run teamplay up to determining role of this teammember.
                cDecisionTree::getInstance().executeTree(gameState, params, robotMemoryStack.at(*it), 0, robotNodes.at(*it));

                // Restore the stashed teammate
                auto determinedRole = teamplay::ownRobotStore::getOwnRobot().getRole();
                teamplay::teamMatesStore::getInstance().restoreStashedRobot(determinedRole);
            }
        }

        std::map<std::string, std::string> myMapParams;

        // Restore own robot, old style:
        cWorldStateFunctions::getInstance().setRobotID(myRobotNr);

        // New style:
        teamplay::ownRobotStore::getInstance().restoreOwnRobot();
        teamplay::teamMatesStore::getTeamMatesIncludingGoalie().remove(teamplay::ownRobotStore::getOwnRobot().getNumber());

        if (overrideState.active)
        {
            // We want to start not on the current gameState, but on the given overrideState tree.
            switch (overrideState.level)
            {
                case overrideLevelEnum::GAMESTATE:
                {
                    gameState = overrideState.gameState;
                    break;
                }
                case overrideLevelEnum::ROLE:
                {
                    gameState = overrideState.role;
                    break;
                }
                case overrideLevelEnum::BEHAVIOR:
                {
                    gameState = overrideState.behavior;
                    myMapParams = overrideState.params;
                    break;
                }
                default:
                {
                    // Do nothing.
                    break;
                }
            }
        }

        // Clear the diagnostics message before executing this iteration of teamplay (remove diag pollution from computing the roles of teammembers)
        cDecisionTree::getInstance().clearDiagMsg();

        if (overrideState.active && (overrideState.level == overrideLevelEnum::ACTION))
        {
            // override action, do not traverse any tree.
            actionEnum action = overrideState.action;
            boost::shared_ptr<cAbstractAction> actionObjectPtr = enumToActionMapping.at(action);
            actionObjectPtr->execute(overrideState.params); // ignore output
        }
        else if (overrideState.active && (overrideState.level == overrideLevelEnum::DISABLED))
        {
            // Do nothing. Teamplay is disabled. :-)
        }
        else
        {
            // Execute the tree (which will recursively call consecutive trees until an action is reached).
            cDecisionTree::getInstance().executeTree(gameState, myMapParams, robotMemoryStack.at(myRobotNr), 0, robotNodes.at(myRobotNr));
        }

        dutyCycleObserver.pokeEnd();
	}
	catch (std::exception &e)
	{
		TRACE_ERROR("Caught exception: ") << e.what();
	}
	catch (...)
	{
	    TRACE_ERROR("Caught unknown exception!");
	}
}

