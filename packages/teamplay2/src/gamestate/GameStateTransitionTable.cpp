// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateTransitionTable.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Own include */
#include "int/gamestate/GameStateTransitionTable.hpp"

/* Other teamplay includes */
#include "cDiagnostics.hpp"
#include "falconsCommonDirs.hpp"

/* System includes */
#include "yaml-cpp/yaml.h"

/* Static definitions */
const static std::string g_gameStateTransitionTableFileName = pathToCodeRepo() + "/config/State_Manager_Transition_Table.yaml";
static std::map< std::pair<teamplay::GameState, RefboxSignalEnum>, teamplay::GameState > g_gameStateTransitionTable;

/* YAML conversion helper structures */
namespace YAML {

struct nestedStringNode
{
    Node node;
    Node nestedNode;
};

template<>
struct convert<nestedStringNode>
{
    static Node encode(const nestedStringNode& rhs)
    {
        Node node;
        node.push_back(rhs.node);
        node.push_back(rhs.nestedNode);
        return node;
    }

    static bool decode(const Node& node, nestedStringNode& rhs)
    {
        if(!node.IsSequence())
        {
            std::cout << "Failure while decoding node. type: " << node.Type() << ", size:" << node.size() << std::endl;
            return false;
        }

        switch(node.size())
        {
        case 1:
            rhs.node = node[0];
            break;

        case 2:
            rhs.node = node[0];
            rhs.nestedNode = node[1];
            break;

        default:
            std::cout << "Node has incorrect size. type: " << node.Type() << ", size:" << node.size() << std::endl;
            return false;
            break;
        }

        return true;
    } /* method decode */
}; /* template struct convert */
} /* namespace YAML */


/* Static functions */
static void readTransitionTable()
{
    YAML::Node inputTable = YAML::LoadFile(g_gameStateTransitionTableFileName);

    std::string gameState_str;
    std::string refboxSignal_str;
    std::string newGameState_str;

    for(YAML::Node::const_iterator it = inputTable.begin(); it != inputTable.end(); it++)
    {
        if(it->size() == 1)
        {
            gameState_str = (*it)["GameState"].as<std::string>();

            refboxSignal_str = "";
            newGameState_str = "";
        }
        else if (it->size() == 2)
        {
            YAML::nestedStringNode refboxNode = it->as<YAML::nestedStringNode>();
            refboxSignal_str = refboxNode.node["RefBoxSignal"].as<std::string>();

            YAML::Node newstateNode = refboxNode.nestedNode;
            newGameState_str = newstateNode[0]["NewGameState"].as<std::string>();
        }
        else
        {
            TRACE_ERROR("Node has an incorrect size.");
            throw std::runtime_error(std::string("Node has an incorrect size."));
        }

        if((!refboxSignal_str.empty()) && (!newGameState_str.empty()))
        {
            teamplay::GameState gameState;
            gameState.fromString(gameState_str);

            // If refbox signal not found, print to stdout first
            if (refboxSignalMapping.find(refboxSignal_str) == refboxSignalMapping.end())
            {
                std::cout << refboxSignal_str << std::endl;
            }

            RefboxSignalEnum refboxSignal = refboxSignalMapping.at(refboxSignal_str);

            teamplay::GameState newGameState;
            newGameState.fromString(newGameState_str);

            g_gameStateTransitionTable.insert( { std::make_pair(gameState, refboxSignal), newGameState } );
        }
    }
}


/* Class implementation */
using namespace teamplay;

GameStateTransitionTable::GameStateTransitionTable()
{
    try
    {
        readTransitionTable();
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Failure while reading GameStateTransitionTable from disk: %s", e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

GameStateTransitionTable::~GameStateTransitionTable()
{
}

const GameState GameStateTransitionTable::calculateNewGameState
      (const GameState& oldGameState, const RefboxSignalEnum& refboxSignal) const
{
    GameState newGameState = g_gameStateTransitionTable.at( std::make_pair(oldGameState, refboxSignal) );

    return newGameState;
}
