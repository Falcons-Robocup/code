// Copyright 2015-2021 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Oct 13, 2015
 *      Author: Jan Feitsma
 */

#define TEAMPLAY_NODENAME ("teamplay_main")

/* System includes */
#include <stdexcept>

/* Other packages includes */
#include "cDiagnostics.hpp"
#include "tracing.hpp"

/* Teamplay includes: input adapters */
#include "int/adapters/ConfigAdapter.hpp"
#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "ext/cRtdbAdapterRefboxSignals.hpp"
#include "int/cTeamplayControlInterface.hpp"

/* Teamplay includes: functionality */
#include "int/cWorldStateFunctions.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/cDecisionTree.hpp"
#include "int/gameStateManager.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/stores/ballStore.hpp"
#include "cDiagnostics.hpp"

namespace {
// The memory stack.
std::map<robotNumber,memoryStackType> robotMemoryStack;
std::map<robotNumber,memoryStackNodes> robotNodes;

// The map containing the parameters.
// Since some behaviors expect the 'role' parameter, and the role is not re-evaluated every time, store the parameters.
std::map<std::string, std::string> myMapParams;

// remember previous gamestate for the role re-evaluation on gamestate change
treeEnum previousGameState = treeEnum::INVALID;
// remember ball possession for the role re-evaluation on ball location known change
bool previousBallLocationKnown = false;
treeEnum ownRobotRole = treeEnum::INVALID;
std::map<robotNumber, treeEnum> teammateRoles;

} // unnamed namespace

// forward declaration
void iterateTeamplay();

int main(int argc, char **argv)
{

    try
    {
        INIT_TRACE("teamplay");

        // setup the refbox signal listener by creating the object once (singleton)
        // (note that this is done via callback, so no periodical update in the loop below)
        cRtdbAdapterRefboxSignals::getInstance();

        // setup the teamplay interface (e.g. for robot control)
        //TODO cControlInterfaceROS controlInterfaceROS;

        // Load the configuration parameters from YAML via RTDB into the configuration store
        auto configAdapter = new ConfigAdapter();
        configAdapter->loadYAML(determineConfig("teamplay"));

        // Load decision trees into memory
        cDecisionTree::getInstance().loadDecisionTrees("");

        // Initialize the gamestate (to neutral stopped)
        teamplay::gameStateStore::getInstance();

        // Initialize the heightmap store
        teamplay::heightMapStore::getInstance();

        /* Loop forever */
        cRTDBInputAdapter::getInstance().waitForRobotState(&iterateTeamplay);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
    }
    catch (...)
    {
        TRACE_ERROR("Caught unknown exception!");
    }
}


void iterateTeamplay()
{
    try
    {
        TRACE_SCOPE("iterateTeamplay", "");

        // duty cycle measurement
        rtime iterationStartTimestamp = ftime::now();

        cRtdbAdapterRefboxSignals::getInstance().update(); // check for new refbox command
        cRTDBInputAdapter::getInstance().getWorldModelData(); // call update functions to retrieve latest worldmodel datafeed

        // Refresh gamestate: check whether a transition from "setpiece execute" to neutral playing is allowed
        teamplay::gameStateManager::getInstance().refreshGameState();

        bool ballLocationKnown = teamplay::ballStore::getInstance().getBall().isLocationKnown();

        // Get override state
        tpOverrideState overrideState;
        cTeamplayControlInterface::getInstance().getOverrideState(overrideState);
        tpOverrideResult overrideResult;
        overrideResult.status = behTreeReturnEnum::INVALID;

        // If override state is gameState, set gameState.
        treeEnum gameState;
        if (overrideState.active && overrideState.level == tpOverrideLevelEnum::GAMESTATE)
        {
            gameState = overrideState.treeValue;

            teamplay::gameStateStore::getInstance().updateGameState(overrideState.treeValue);
        }
        else
        {
            gameState = teamplay::gameStateStore::getInstance().getGameState_treeEnum();
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

        bool ownRobotRoleIsDuplicate = false;
        if (ownRobotRole != treeEnum::INVALID)
        {
            if (!teamplay::role(ownRobotRole).isSafeOnMultipleRobots())
            {
                auto teammates = teamplay::robotStore::getInstance().getAllRobotsExclOwnRobot();
                for (auto it = teammates.begin(); it != teammates.end(); ++it)
                {
                    std::string teammatesRole = cRTDBInputAdapter::getInstance().getRole(it->getNumber());
                    if (teamplay::role(ownRobotRole).str() == teammatesRole)
                    {
                        tprintf("Duplicate role (%s) detected with robot %d", teammatesRole.c_str(), it->getNumber());
                        ownRobotRoleIsDuplicate = true;
                    }
                }
            }
        }

        // Determine the role for my teammembers.
        // My decisions depend on whether some teammembers are present. (e.g., attacker assist)

        // TODO - should we clear this memory before recomputing? Or at the end of an iteration? If a robot drops out, his role will remain in memory.
        // However, all functions that use "getRobotRole" iterate on active_robots, so that should always be fine.

        auto teammates = teamplay::robotStore::getInstance().getAllRobotsExclOwnRobot();
        for (auto it = teammates.begin(); it != teammates.end(); ++it)
        {
            std::map<std::string, std::string> params;
            auto teammate_number = it->getNumber();

            if (robotMemoryStack.find(teammate_number) == robotMemoryStack.end())
            {
                // Add memory of this robot
                robotMemoryStack.insert( std::make_pair(teammate_number, memoryStackType()) );
            }
            if (robotNodes.find(teammate_number) == robotNodes.end())
            {
                // Add nodes of this robot
                robotNodes.insert( std::make_pair(teammate_number, memoryStackNodes()) );
            }

            // Set robot ID at WSF so that any questions asked towards WSF are as if it was this robot asking it, old style:
            cWorldStateFunctions::getInstance().setRobotID(teammate_number);

            // New style:
            teamplay::robotStore::getInstance().exchangeOwnRobotWith(teammate_number);

            // Cold start -- make sure the current teammate is in the teammateRoles
            if (teammateRoles.find(teammate_number) == teammateRoles.end())
            {
                teammateRoles.insert( std::make_pair(teammate_number, treeEnum::R_ROBOT_STOP) );
            }

            // Only re-evaluate the role of this robot when:
            // - gamestate changes, but not to setpiece execute or:
            // - ball_location_known changes (find ball or lose sight of ball) during setpiece prepare
            if (  (gameState != previousGameState && !teamplay::gameStateStore::getInstance().getGameState().isExecuteSetPiece())
                    || (ballLocationKnown != previousBallLocationKnown && teamplay::gameStateStore::getInstance().getGameState().isPrepareSetPiece())
                    || ((gameState != previousGameState) && teamplay::gameStateStore::getInstance().getGameState().isParkingSetPiece())
                    || ownRobotRoleIsDuplicate
                )
            {
                TRACE("Re-evaluating role for team member r") << std::to_string(teammate_number);
                // Run teamplay up to determining role of this teammember.
                cDecisionTree::getInstance().executeTree(gameState, params, robotMemoryStack.at(teammate_number), 0, robotNodes.at(teammate_number));

                teammateRoles[teammate_number] = teamplay::robotStore::getInstance().getOwnRobot().getRole();
            }
            else
            {
                teamplay::robotStore::getInstance().setOwnRobotRole(teammateRoles[teammate_number]);
            }

            // Restore the own robot, new style:
            teamplay::robotStore::getInstance().undoExchange();
        }

        // Restore own robot, old style:
        cWorldStateFunctions::getInstance().setRobotID(myRobotNr);

        // Calculate all heightmaps
        tprintf("start heightmap precalculation");  // temporary diagnostics
        teamplay::heightMapStore::getInstance().precalculateAll();
        tprintf("finish heightmap precalculation");  // temporary diagnostics

        // Clear the diagnostics message before executing this iteration of teamplay (remove diag pollution from computing the roles of teammembers)
        // Add the gamestate to the diagnostics message -- used to check consistency between robots.
        cDecisionTree::getInstance().clearDiagMsg();


        // normal flow: overrideState inactive, or when overrideState in gameState.
        if (!overrideState.active || (overrideState.active && overrideState.level == tpOverrideLevelEnum::GAMESTATE))
        {
            // Only re-evaluate the role of this robot when:
            // - gamestate changes, but not to setpiece execute or:
            // - ball_location_known changes (find ball or lose sight of ball) during setpiece prepare
            if (  (gameState != previousGameState && !teamplay::gameStateStore::getInstance().getGameState().isExecuteSetPiece())
                    || (ballLocationKnown != previousBallLocationKnown && teamplay::gameStateStore::getInstance().getGameState().isPrepareSetPiece())
                    || ((gameState != previousGameState) && teamplay::gameStateStore::getInstance().getGameState().isParkingSetPiece())
                    || ownRobotRoleIsDuplicate
                )
            {
                TRACE("Re-evaluating role for robot ") << std::to_string(myRobotNr);
                TRACE_SCOPE("RE-EVALUATE ROLE", "");

                // Execute the tree (which will recursively call consecutive trees until a role is reached).
                myMapParams.clear();
                cDecisionTree::getInstance().executeTree(gameState, myMapParams, robotMemoryStack.at(myRobotNr), 0, robotNodes.at(myRobotNr));

                robotMemoryStack.at(myRobotNr).clear();
                robotNodes.at(myRobotNr).clear();

                ownRobotRole = teamplay::robotStore::getInstance().getOwnRobot().getRole();

                previousGameState = gameState;
                previousBallLocationKnown = ballLocationKnown;
            }
            else
            {
                teamplay::robotStore::getInstance().setOwnRobotRole(ownRobotRole);
            }

            cDecisionTree::getInstance().diagMsg.trees.push_back( treeEnumToStr(gameState) );

            // Having the role of this robot, execute the role.
            treeEnum treeToExecute = teamplay::robotStore::getInstance().getOwnRobot().getRole();
            cDecisionTree::getInstance().executeTree(treeToExecute, myMapParams, robotMemoryStack.at(myRobotNr), 0, robotNodes.at(myRobotNr));
        }
        else
        {
            // overrideState is active. Either set to role, behavior or action.

            if (overrideState.level == tpOverrideLevelEnum::DISABLED)
            {
                // Do nothing. Teamplay is disabled. :-)
            }
            else if (overrideState.level == tpOverrideLevelEnum::TP_ACTION)
            {
                // override Teamplay action, do not traverse any tree.
                tpActionEnum action = overrideState.tpAction;
                boost::shared_ptr<cAbstractAction> actionObjectPtr = enumToActionMapping.at(action);
                overrideResult.status = actionObjectPtr->execute(overrideState.params);
            }
            else if (overrideState.level == tpOverrideLevelEnum::MP_ACTION)
            {
                // override MotionPlanning action, do not traverse any tree.
                actionResult mpResult = cRTDBOutputAdapter::getInstance().getMPClient().executeAction(overrideState.mpAction);
                behTreeReturnEnum tpResult = behTreeReturnEnum::INVALID;
                switch(mpResult.result)
                {
                    case actionResultTypeEnum::INVALID:
                    {
                        tpResult = behTreeReturnEnum::INVALID;
                        break;
                    }
                    case actionResultTypeEnum::PASSED:
                    {
                        tpResult = behTreeReturnEnum::PASSED;
                        break;
                    }
                    case actionResultTypeEnum::FAILED:
                    {
                        tpResult = behTreeReturnEnum::FAILED;
                        break;
                    }
                    case actionResultTypeEnum::RUNNING:
                    {
                        tpResult = behTreeReturnEnum::RUNNING;
                        break;
                    }
                }
                overrideResult.status = tpResult;
            }
            else
            {
                // execute tree of behavior / role
                overrideResult.status = cDecisionTree::getInstance().executeTree(overrideState.treeValue, overrideState.params, robotMemoryStack.at(myRobotNr), 0, robotNodes.at(myRobotNr));
                tprintf("role/behavior override result=%s", enum2str(overrideResult.status));
            }
        }

        // if applicable, write result back into RTDB for test interfacing
        if (overrideState.active)
        {
            tprintf("writing override result=%s", enum2str(overrideResult.status));
            cTeamplayControlInterface::getInstance().setOverrideResult(overrideResult);
        }

        // duty cycle measurement
        double elapsed = double(ftime::now() - iterationStartTimestamp);
        double nominalFrequency = 30.0;
        double heartBeatDuration = 1.0 / nominalFrequency;
        double threshold = heartBeatDuration * 0.9; // 90% of a heartbeat
        threshold = 0.1; // for now we use a relaxed spec, to detect spikes
        if (elapsed > threshold)
        {
            TRACE_WARNING("tick duration too long: %.2fs", elapsed);
        }
        rtime tNow;

    }
    catch (std::exception &e)
    {
        tprintf("exception catch 1 %s", e.what());
        TRACE_ERROR("Caught exception: %s", e.what());
    }
    catch (...)
    {
        tprintf("exception catch 2");
        TRACE_ERROR("Caught unknown exception!");
    }

    WRITE_TRACE;
}

