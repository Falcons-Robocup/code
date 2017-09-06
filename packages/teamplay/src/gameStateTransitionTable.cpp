 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameStateTransitionTable.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Own include */
#include "int/gameStateTransitionTable.hpp"

/* Other teamplay includes */
#include "int/types/cRefboxSignalTypesFunctions.hpp"
#include "int/utilities/trace.hpp"

/* System includes */
#include "yaml-cpp/yaml.h"

/* Static definitions */
const static std::string g_gameStateTransitionTableFileName = "/home/robocup/falcons/code/config/State_Manager_Transition_Table.yaml";
static treeEnum g_gameStateTransitionTable[(size_t)treeEnum::SIZE_OF_ENUM][(size_t)refboxSignalEnum::SIZE_OF_ENUM];

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
            treeEnum gameState = gameStateEnumMapping.at(gameState_str);
            refboxSignalEnum refboxSignal = refBoxSignalStrToEnum(refboxSignal_str);
            treeEnum newGameState = gameStateEnumMapping.at(newGameState_str);

            g_gameStateTransitionTable[(size_t)gameState][(size_t)refboxSignal] = newGameState;
        }
    }
}


/* Class implementation */
using namespace teamplay;

gameStateTransitionTable::gameStateTransitionTable()
{
    try
    {
        readTransitionTable();
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Failure while reading gameStateTransitionTable from disk: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

gameStateTransitionTable::~gameStateTransitionTable()
{
}

const gameState gameStateTransitionTable::calculateNewGameState
      (const gameState& oldGameState, const refboxSignalEnum& refboxSignal) const
{
    treeEnum newGameState = g_gameStateTransitionTable[(size_t)oldGameState.toTreeEnum()][(size_t)refboxSignal];
    return gameState(newGameState);
}
