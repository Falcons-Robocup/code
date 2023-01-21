// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Oct 13, 2015
 *      Author: Jan Feitsma
 */

/* System includes */
#include <stdexcept>

/* BehaviorTree includes */
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"

/* Other packages includes */
#include "MTPAdapter.hpp"
#include "cDiagnostics.hpp"
#include "tracing.hpp"
#include "tpOverrideLevelEnum.hpp"

/* Teamplay includes: input adapters */
#include "int/adapters/RTDBDiagnosticsAdapter.hpp"
#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/adapters/RTDBOutputAdapter.hpp"
#include "int/adapters/RTDBAdapterRefboxSignals.hpp"

#include "int/gamestate/GameStateManager.hpp"

#include "int/plays/PlaySelection.hpp"
#include "int/plays/RoleAssignment.hpp"

#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/GameStateStore.hpp"
#include "int/stores/HeightMapStore.hpp"

//#include "int/cTeamplayControlInterface.hpp"

/* Teamplay includes: functionality */
//#include "int/cWorldStateFunctions.hpp"
//#include "int/cWorldModelInterface.hpp"
//#include "int/cDecisionTree.hpp"
//#include "int/GameStateManager.hpp"
//#include "int/stores/GameStateStore.hpp"
//#include "int/stores/HeightMapStore.hpp"
//#include "int/stores/RobotStore.hpp"
//#include "int/stores/BallStore.hpp"

#include "int/types/BehaviorTreeTypes.hpp"

//////////////////
// Action Nodes //
//////////////////
#include "int/actionnodes/GetBall.hpp"
#include "int/actionnodes/GoalKeeper.hpp"
#include "int/actionnodes/InterceptBall.hpp"
#include "int/actionnodes/Move.hpp"
#include "int/actionnodes/PositionBeforePOI.hpp"
#include "int/actionnodes/PositionBehindPOI.hpp"
#include "int/actionnodes/PositionWithHeightmap.hpp"
#include "int/actionnodes/SetRole.hpp"
#include "int/actionnodes/ShootToPOI.hpp"
#include "int/actionnodes/Stop.hpp"
#include "int/actionnodes/TurnAwayFromOpponent.hpp"

///////////////////
// Control Nodes //
///////////////////
#include "int/controlnodes/SimpleQueries.hpp"
#include "int/controlnodes/AdvancedQueries.hpp"
#include "int/controlnodes/SequenceStarAsyncNode.hpp"

/////////////////////
// Decorator Nodes //
/////////////////////
#include "int/decoratornodes/AvoidPOI.hpp"

// forward declaration
void executeGameState();
void executeRole();
tpOverrideLevelEnum evaluateTeamplayOverride();
void publishTeamplayOverrideResult(BT::NodeStatus resultStatus);

void waitForRobotRoles();
boost::thread executeRoleThread;

// We use the BehaviorTreeFactory to register our custom nodes
BT::BehaviorTreeFactory factory;
BT::Tree mainTree;
BT::Tree setPieceTree;
BT::PublisherZMQ* zmqPublisher;
BT::FileLogger* treeFileLogger;
BT::MinitraceLogger* minitraceLogger;

// MixedTeamProtocol
MixedTeamProtocolAdapter *mtpAdapter = NULL;

// duty cycle measurement
rtime iterationStartTimestamp;

// keep track of gamestate changes
teamplay::GameState prevGameState;
bool newRolesAvailable = false;

void registerNodes(BT::BehaviorTreeFactory& btFactory)
{
    // Actions
    btFactory.registerNodeType<GetBall>("GetBall");
    btFactory.registerNodeType<GoalKeeper>("GoalKeeper");
    btFactory.registerNodeType<InterceptBall>("InterceptBall");
    btFactory.registerNodeType<Move>("Move");
    btFactory.registerNodeType<PositionBeforePOI>("PositionBeforePOI");
    btFactory.registerNodeType<PositionBehindPOI>("PositionBehindPOI");
    btFactory.registerNodeType<PositionWithHeightmap>("PositionWithHeightmap");
    btFactory.registerNodeType<SetRole>("SetRole");
    btFactory.registerNodeType<ShootToPOI>("ShootToPOI");
    btFactory.registerNodeType<TurnAwayFromOpponent>("TurnAwayFromOpponent");

    // Simple Queries
    registerNodes_SimpleQueries(btFactory);

    // Advanced Queries
    registerNodes_AdvancedQueries(btFactory);

    // SequenceStarAsyncNode
    registerNode_SequenceStarAsyncNode(btFactory);

    // Decorator Nodes
    btFactory.registerNodeType<AvoidPOI>("AvoidPOI");
}

int main(int argc, char **argv)
{

    try
    {
        INIT_TRACE("teamplay2");

        TRACE("Initializing..");

        // setup the refbox signal listener by creating the object once (singleton)
        // (note that this is done via callback, so no periodical update in the loop below)
        RTDBAdapterRefboxSignals::getInstance();

        // Setup MTP
        mtpAdapter = new MixedTeamProtocolAdapter();

        // Load the Configuration parameters from YAML via RTDB into the Configuration store
        teamplay::ConfigurationStore::getInstance();

        // register custom nodes to BT Factory
        registerNodes(factory);

        // setup RoleAssignment
        teamplay::RoleAssignment::getInstance().setBTFactory(factory);

        // setup MTP 
        teamplay::RoleAssignment::getInstance().setMTP(mtpAdapter);
        tpRTDBInputAdapter::getInstance().setMTP(mtpAdapter);
        RTDBAdapterRefboxSignals::getInstance().setMTP(mtpAdapter);

        // populate trees from XML
        setPieceTree = factory.createTreeFromFile(pathToTeamplayDataRepo() + "/SetPiece.xml");
        mainTree = factory.createTreeFromFile(pathToTeamplayDataRepo() + "/ActionTree.xml");

        // BehaviorTree logging.

        // ZMQ logging for live visualization
        //zmqPublisher = new BT::PublisherZMQ(mainTree, 25, 1600 + getRobotNumber(), 1700 + getRobotNumber());
        
        // File logging for offline replay
        std::ostringstream oss;
        oss << "/var/tmp/tplogging_r" << getRobotNumber() << ".fbl";
        treeFileLogger = new BT::FileLogger(mainTree, oss.str().c_str());

        // Minitrace logging
        // if uncommenting this, dont forget to `minitraceLogger.flush()`
        //std::ostringstream oss;
        //oss << "/var/tmp/tplogging_r" << getRobotNumber() << ".json";
        //minitraceLogger = new BT::MinitraceLogger(mainTree, oss.str().c_str());

        // Initialize the heightmap store
        teamplay::HeightMapStore::getInstance();

        // Start 'waitForPut' in two threads.
        //
        // TP_HEARTBEAT -> executeGameState in the main thread
        // -> grabs the latest data from worldmodel
        // -> updates the current gamestate
        // -> lowest active robot id decides the new roles for all robots
        // -> other robots read the robot roles from the lowest active robot id
        // -> (re)publishes ROBOT_ROLES
        //
        // ROBOT_ROLES -> executeRole in a new thread
        // -> grabs the latest data from worldmodel
        // -> precalculates the heightmaps
        // -> uses the robot role to determine an action
        // -> publishes ACTION
        //
        executeRoleThread = boost::thread(&waitForRobotRoles);
        tpRTDBInputAdapter::getInstance().waitForTPHeartbeat(&executeGameState);
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

void executeGameState()
{
    try
    {
        TRACE_SCOPE("executeGameState", "");

        // duty cycle measurement
        iterationStartTimestamp = ftime::now();

        RTDBAdapterRefboxSignals::getInstance().update(); // check for new refbox command
        tpRTDBInputAdapter::getInstance().getWorldModelData(); // call update functions to retrieve latest worldmodel datafeed

        // Refresh gamestate: check whether a transition from "setpiece execute" to neutral playing is allowed
        teamplay::GameStateManager::getInstance().refreshGameState();

        // Assign the roles, triggering the ActionTree
        teamplay::RoleAssignment::getInstance().assignRoles();

        // if (newRolesAvailable)
        {
            executeRole();
            newRolesAvailable = false;
        }
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

}

void setNewRolesAvailable()
{
    newRolesAvailable = true;
}

void waitForRobotRoles()
{
    INIT_TRACE_THREAD("waitForRobotRoles");
    tpRTDBInputAdapter::getInstance().waitForRobotRoles(&setNewRolesAvailable);
}

void executeRole()
{
    try
    {
        TRACE_SCOPE("executeRole", "");

        //tpRTDBInputAdapter::getInstance().getWorldModelData(); // call update functions to retrieve latest worldmodel datafeed

        // Reset the precalculation for all heightmaps
        teamplay::HeightMapStore::getInstance().resetHeightmapPrecalculations();

        // evaluate teamplay override interface
        // this interface is used for testing purposes
        tpOverrideLevelEnum overrideActive = evaluateTeamplayOverride();

        // SPUX: no longer need to halt trees now that executeRole is in (sequence) with executeGameState
        // When gamestate changes, halt trees.
        // This means that all 'running' nodes are reset, and start over.
        // if (prevGameState != teamplay::GameStateStore::getInstance().getGameState())
        // {
        //     setPieceTree.haltTree();
        //     mainTree.haltTree();
        // }

        BT::NodeStatus resultStatus = BT::NodeStatus::IDLE;

        if (overrideActive != tpOverrideLevelEnum::INVALID)
        {
            //overrideActive == TRUE

            if (overrideActive == tpOverrideLevelEnum::ROLE)
            {
                TRACE("tpOverride: Ticking Main Tree -- overruled role");
                resultStatus = mainTree.tickRoot();
            }
            else if (overrideActive == tpOverrideLevelEnum::GAMESTATE)
            {
                if (teamplay::GameStateStore::getInstance().getGameState().isSetPiece())
                {
                    TRACE("tpOverride: Ticking SetPiece Tree");
                    resultStatus = setPieceTree.tickRoot();
                }
                else
                {
                    TRACE("tpOverride: Ticking Main Tree -- overruled gamestate");
                    resultStatus = mainTree.tickRoot();
                }
            }

            publishTeamplayOverrideResult(resultStatus);
        }
        else
        {
            //overrideActive == FALSE

            // Traverse Behavior Tree
            if (teamplay::GameStateStore::getInstance().getGameState().isStopped())
            {
                TRACE_SCOPE("Action Stop", "");
                Stop stopAction;
                stopAction.stop();
            }
            else if (teamplay::GameStateStore::getInstance().getGameState().isSetPiece())
            {
                TRACE_SCOPE("SetPiece Tree", "");
                resultStatus = setPieceTree.tickRoot();
            }
            else
            {
                TRACE_SCOPE("Main Tree", "");
                resultStatus = mainTree.tickRoot();
            }

        }

        // duty cycle measurement
        double elapsed = double(ftime::now() - iterationStartTimestamp);
        double nominalFrequency = teamplay::ConfigurationStore::getInstance().getConfiguration().getNominalFrequency();
        double heartBeatDuration = 1.0 / nominalFrequency;
        double threshold = heartBeatDuration * 0.5; // 50% of a heartbeat
        if (elapsed > threshold)
        {
            TRACE_WARNING("tick duration too long: %.2fs", elapsed);
        }

        prevGameState = teamplay::GameStateStore::getInstance().getGameState();

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

    // Call update functions to set the latest diagnostic datafeed for trees
    tpRTDBDiagnosticsAdapter::getInstance().setDiagnosticsData();

    WRITE_TRACE;
}

tpOverrideLevelEnum evaluateTeamplayOverride()
{
    // Get override string from RtDB
    int myRobotId = getRobotNumber();
    FalconsRTDB* rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(myRobotId, getTeamChar());
    int ageMs = 0;
    T_TP_OVERRIDE_STATE overrideState;
    int r = rtdb->get(TP_OVERRIDE_STATE, &overrideState, ageMs, myRobotId);

    if (r == RTDB2_SUCCESS && ageMs < 200 && overrideState.active)
    {
        //// Role
        teamplay::RoleEnum r;
        r = teamplay::str2enum(overrideState.overrideStr);
        if (r != teamplay::RoleEnum::INVALID)
        {
            teamplay::RobotStore::getInstance().setOwnRobotRole(r);
            // If we found a Role, return. Otherwise, check if the string is a GameState.
            return tpOverrideLevelEnum::ROLE;
        }

        //// GameState
        teamplay::GameState gs;
        gs.fromString(overrideState.overrideStr);
        if (gs.isValidGameState())
        {
            teamplay::GameStateStore::getInstance().updateGameState(gs);
            // If we found a GameState, return. Otherwise, throw an error.
            return tpOverrideLevelEnum::GAMESTATE;
        }
        else
        {
            std::string err = "Unknown override string found:" + overrideState.overrideStr;
            throw std::runtime_error(err);
        }
    }
    else
    {
        return tpOverrideLevelEnum::INVALID;
    } 
}

void publishTeamplayOverrideResult(BT::NodeStatus resultStatus)
{
    // Write override result status to RtDB
    int myRobotId = getRobotNumber();
    FalconsRTDB* rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(myRobotId, getTeamChar());

    T_TP_OVERRIDE_RESULT overrideResult;
    switch(resultStatus)
    {
        case BT::NodeStatus::RUNNING:
        {
            overrideResult.status = behTreeReturnEnum::RUNNING;
            break;
        }
        case BT::NodeStatus::SUCCESS:
        {
            overrideResult.status = behTreeReturnEnum::PASSED;
            break;
        }
        case BT::NodeStatus::FAILURE:
        {
            overrideResult.status = behTreeReturnEnum::FAILED;
            break;
        }
        case BT::NodeStatus::IDLE:
        {
            overrideResult.status = behTreeReturnEnum::PASSED;
            break;
        }
    }

    int r = rtdb->put(TP_OVERRIDE_RESULT, &overrideResult);

    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Failed to write TP_OVERRIDE_RESULT");
    }
}