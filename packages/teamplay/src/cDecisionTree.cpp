// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDecisionTree.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: Erik Kouters
 */
#include "int/cDecisionTree.hpp"
#include "int/types/cDecisionTreeTypes.hpp"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pwd.h>
#include <stdexcept>
#include <unistd.h>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/assign/ptr_map_inserter.hpp>

#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/types/cGameStateTypes.hpp"
#include "int/types/cRefboxSignalTypes.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/types/cWorldStateFunctionTypes.hpp"
#include "int/types/cWorldStateFunctionTypesFunctions.hpp"

/* Teamplay includes: actions */
#include "int/actions/cActionStop.hpp"
#include "int/actions/cActionShoot.hpp"
#include "int/actions/cActionPass.hpp"
#include "int/actions/cActionPositionBeforePOI.hpp"
#include "int/actions/cActionPositionBehindPOI.hpp"
#include "int/actions/cActionPositionForOppSetpiece.hpp"
#include "int/actions/cActionPositionForOwnSetpiece.hpp"
#include "int/actions/cActionGetBall.hpp"
#include "int/actions/cActionGoalKeeper.hpp"
#include "int/actions/cActionInterceptBall.hpp"
#include "int/actions/cActionMove.hpp"
#include "int/actions/cActionMoveToFreeSpot.hpp"
#include "int/actions/cActionSuccess.hpp"
#include "int/actions/cActionAvoidPOI.hpp"
#include "int/actions/cActionDefendPenaltyArea.hpp"
#include "int/actions/cActionTurnAwayFromOpponent.hpp"
#include "int/actions/cActionDefendAttackingOpponent.hpp"
#include "int/actions/cActionDribble.hpp"

#include "cDiagnostics.hpp"

using namespace teamplay;

// define header variable (cActionMapping.hpp). Populated in the constructor of cDecisionTree()
boost::ptr_map<tpActionEnum, boost::shared_ptr<cAbstractAction> > enumToActionMapping;

cParsedNode::cParsedNode(const boost::property_tree::ptree &root)
{
    _root = root;

    // Parse node's id
    std::string strId = root.get<std::string>("id");
    _id = boost::lexical_cast<boost::uuids::uuid>(strId);

    _name = root.get<std::string>("name");
    _title = root.get<std::string>("title");

    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, root.get_child("properties"))
    {
        std::string propertyKey = v.first;
        std::string propertyVal = v.second.get_value<std::string>();
        _mapProperties.insert(std::make_pair(propertyKey, propertyVal));
    }

    boost::optional<const boost::property_tree::ptree&> children = root.get_child_optional("children");
    if (children)
    {
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, root.get_child("children"))
        {
            _children.push_back(boost::lexical_cast<boost::uuids::uuid>(v.second.get_value<std::string>()));
        }
    }

    // Determine _nodeType
    if (_name == "MemSequence")
    {
        _nodeType = nodeEnum::MEMSEQUENCE;
    }
    else if (_name == "Sequence")
    {
        _nodeType = nodeEnum::SEQUENCE;
    }
    else if (_name == "Priority")
    {
        _nodeType = nodeEnum::SELECTOR;
    }
    else if (_name == "MemPriority")
    {
        _nodeType = nodeEnum::MEMSELECTOR;
    }
    else if (_name == "parameterCheck")
    {
        _nodeType = nodeEnum::PARAMETER_CHECK;
    }
    else if (_mapProperties.find(std::string("wsf")) != _mapProperties.end())
    {
        if (_mapProperties.find(std::string("hasMemory")) != _mapProperties.end())
        {
            std::string hasMemory = _mapProperties.at(std::string("hasMemory"));
            if (hasMemory.compare("false") == 0)
            {
                _nodeType = nodeEnum::WORLDSTATE_FUNCTION;
            }
            else if (hasMemory.compare("true") == 0)
            {
                _nodeType = nodeEnum::MEMWORLDSTATE_FUNCTION;
            }
        }

        std::string wsf = _mapProperties.at(std::string("wsf"));

        // Resolve worldStateFunction
        // Convert: string -> worldState function Enum -> worldStateFunction
        try
        {
            worldStateFunctionEnum worldStateFunctionEnumVal = worldStateFunctionMappingStrToEnum.at(wsf);

            try
            {
                _wsf = worldStateFunctionMappingEnumToFunc.at(worldStateFunctionEnumVal);
            }
            catch (std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
                throw std::runtime_error("Node '" + _name + "' identified as worldStateFunction, but worldStateFunction '" + wsf + "' is not found in worldStateFunctionMappingEnumToFunc.");
            }
        }
        catch (std::exception &e)
        {
            TRACE_ERROR("Caught exception: %s", e.what());
            throw std::runtime_error("Node '" + _name + "' identified as worldStateFunction, but worldStateFunction '" + wsf + "' is not found in worldStateFunctionMappingStrToEnum.");
        }
    }
    else
    {
        // Can be action (act), behavior (beh), role (rol)
        size_t strPos = _name.find_first_of(":"); // -> act:shoot
        std::string strNodeType = _name.substr(0, strPos); // -> act

        // If act, beh or rol, overwrite name with the type extracted.
        // _name = rol:stop -> stop
        if (strNodeType.compare("act") == 0)
        {
            _nodeType = nodeEnum::ACTION;
            _name = _name.substr(strPos + 1);

            // Resolve action
            try
            {
                tpActionEnum action = actionMapping.at(_name);

                try
                {
                    _action = enumToActionMapping.at(action);
                }
                catch(std::exception &e)
                {

                    TRACE_ERROR("Caught exception: %s", e.what());
                    throw std::runtime_error("Node '" + _name + "' identified as action, but action is not found in enumToActionMapping.");
                }
            }
            catch(std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
                throw std::runtime_error("Node '" + _name + "' identified as action, but action is not found in actionMapping.");
            }

            //// Resolve action parameters and values
            // First check for each parameter that it exists in the list of allowed parameters: _action->actionParameters
            // Secondly, if the parameter is allowed, check that its value is in the list of allowed values.
            std::map<std::string, std::string>::const_iterator itParam;
            for (itParam = _mapProperties.begin(); itParam != _mapProperties.end(); ++itParam)
            {
                // if the action's parameter is in the list of allowed parameters
                // if itParam->first in actionParameters.keys()
                if (_action->_actionParameters.find( itParam->first ) != _action->_actionParameters.end())
                {
                    // if the parameter's value is in the list of allowed values
                    // if itParam->second not in actionParameters[itParam->first].values()
                    std::vector<std::string> allowedValues = _action->_actionParameters.at(itParam->first).first;
                    bool optional = _action->_actionParameters.at(itParam->first).second;

//                    std::vector<std::string>::const_iterator it;
//                    for (it = allowedValues.begin(); it != allowedValues.end(); ++it)
//                    {
//                        std::cout << *it << std::endl;
//                    }

                    if (std::find(allowedValues.begin(), allowedValues.end(), itParam->second) == allowedValues.end())
                    {
                        // The parameter value is not in the list of allowed parameters.
                        // Corner cases:
                        // poi          : Any POI from environment is allowed
                        // area         : Any Area from environment is allowed
                        // emptyValue   : The emptyValue is specified, meaning this parameter is intentionally left empty. Only allowed when optional.
                        // float        : The value is a float. Try to convert to float.
                        // bool         : The bool should be "true" or "false".

                        bool foundAllowedValue = false;

                        // if the value is the emptyValue, and the parameter is optional.
                        if ( (itParam->second.compare(emptyValue) == 0) && optional)
                        {
                            foundAllowedValue = true;
                        }
                        // if poi in allowedValues:
                        else if (std::find(allowedValues.begin(), allowedValues.end(), poiValue) != allowedValues.end())
                        {
                            // Validate environment value.
                            if( fieldDimensionsStore::getFieldDimensions().isValidPOI(itParam->second) )
                            {
                                foundAllowedValue = true;
                            }
                        }
                        // if area in allowedValues:
                        else if (std::find(allowedValues.begin(), allowedValues.end(), areaValue) != allowedValues.end())
                        {
                            // Validate environment value.
                            if( fieldDimensionsStore::getFieldDimensions().isValidArea(itParam->second) )
                            {
                                foundAllowedValue = true;
                            }
                        }
                        // if float in allowedValues:
                        else if (std::find(allowedValues.begin(), allowedValues.end(), "float") != allowedValues.end())
                        {
                            // Parse to float, if possible.
                            try
                            {
                                std::stof(itParam->second); // if not a valid flow, will throw illegal argument exception
                                foundAllowedValue = true;
                            }
                            catch(std::exception &e)
                            {
                                // Nope, definitely not a float.
                                TRACE_ERROR("Caught exception: %s", e.what());
                                throw std::runtime_error(std::string("cParsedNode::cParsedNode Linked to: ") + e.what());
                            }
                        }
                        // if bool in allowedValues:
                        else if (std::find(allowedValues.begin(), allowedValues.end(), "bool") != allowedValues.end())
                        {
                            // Must be "true" or "false"
                            if (itParam->second.compare("true") == 0)
                            {
                                foundAllowedValue = true;
                            }
                            else if (itParam->second.compare("false") == 0)
                            {
                                foundAllowedValue = true;
                            }
                        }

                        // None of the corner cases were correct, so throwing exception.
                        if (!foundAllowedValue)
                        {
                            throw std::runtime_error("Node '" + _name + "' has the parameter '" + itParam->first + "' with value '" + itParam->second + "', but the value is not allowed as specified in actionParameters.");
                        }
                    }
                }
                else
                {
                    throw std::runtime_error("Node '" + _name + "' has the parameter '" + itParam->first + "', but this is not specified in actionParameters.");
                }
            }
        }
        else if (strNodeType.compare("beh") == 0)
        {
            _nodeType = nodeEnum::BEHAVIOR;
            _name = _name.substr(strPos + 1);

            // Resolve behavior
            try
            {
                _behavior = treeEnumMapping.at(_name);
            }
            catch(std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
                throw std::runtime_error("Node '" + _name + "' identified as behavior, but behavior is not found in treeEnumMapping.");
            }
        }
        else if (strNodeType.compare("rol") == 0)
        {
            _nodeType = nodeEnum::ROLE;
            _name = _name.substr(strPos + 1);

            // Resolve role
            try
            {
                _role = treeEnumMapping.at(_name);
            }
            catch(std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
                throw std::runtime_error("Node '" + _name + "' identified as role, but role is not found in treeEnumMapping.");
            }
        }
        else
        {
            _nodeType = nodeEnum::INVALID;
            throw std::runtime_error("Unknown node found: '" + _name + "'.");
        }
    }
}

cParsedTree::cParsedTree(const boost::property_tree::ptree &root)
{
    _root = root;

    // Store tree uuid
    std::string strId = root.get<std::string>("id");
    _id = boost::lexical_cast<boost::uuids::uuid>(strId);

    // Store root node uuid
    std::string strRootNode = root.get<std::string>("root");
    _rootNode = boost::lexical_cast<boost::uuids::uuid>(strRootNode);

    // Iterate nodes, store as cParsedNodes in _nodeMapping.
    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, root.get_child("nodes"))
    {
        std::string strNodeName = v.first;
        _nodeMapping.insert(std::make_pair(boost::lexical_cast<boost::uuids::uuid>(strNodeName), cParsedNode(v.second)));
    }
}

/*!
 * \brief cDecisionTree is responsible for loading all decision trees into memory and supplying the functions to execute it.
 *
 * There are currently two types of decision trees.
 * 1. gameState -> role
 * 2. role -> behavior
 */
cDecisionTree::cDecisionTree()
{
    TRACE_FUNCTION("");
    boost::assign::ptr_map_insert( enumToActionMapping )
        ( tpActionEnum::INVALID, boost::make_shared<cActionStop>() )
        ( tpActionEnum::SUCCESS, boost::make_shared<cActionSuccess>() )
        ( tpActionEnum::MOVE, boost::make_shared<cActionMove>() )
        ( tpActionEnum::STOP, boost::make_shared<cActionStop>() )
        ( tpActionEnum::SHOOT, boost::make_shared<cActionShoot>() )
        ( tpActionEnum::PASS, boost::make_shared<cActionPass>() )
        ( tpActionEnum::POSITION_BEFORE_POI, boost::make_shared<cActionPositionBeforePOI>() )
        ( tpActionEnum::POSITION_BEHIND_POI, boost::make_shared<cActionPositionBehindPOI>() )
        ( tpActionEnum::POSITION_FOR_OPP_SETPIECE, boost::make_shared<cActionPositionForOppSetpiece>() )
        ( tpActionEnum::POSITION_FOR_OWN_SETPIECE, boost::make_shared<cActionPositionForOwnSetpiece>() )
        ( tpActionEnum::GET_BALL, boost::make_shared<cActionGetBall>() )
        ( tpActionEnum::GOALKEEPER, boost::make_shared<cActionGoalKeeper>() )
        ( tpActionEnum::INTERCEPT_BALL, boost::make_shared<cActionInterceptBall>() )
        ( tpActionEnum::MOVE_TO_FREE_SPOT, boost::make_shared<cActionMoveToFreeSpot>())
        ( tpActionEnum::AVOID_POI,  boost::make_shared<cActionAvoidPOI>())
        ( tpActionEnum::DEFEND_PENALTY_AREA,  boost::make_shared<cActionDefendPenaltyArea>())
        ( tpActionEnum::TURN_AWAY_FROM_OPPONENT,  boost::make_shared<cActionTurnAwayFromOpponent>())
        ( tpActionEnum::DEFEND_ATTACKING_OPPONENT,  boost::make_shared<cActionDefendAttackingOpponent>())
        ( tpActionEnum::DRIBBLE,  boost::make_shared<cActionDribble>())
        ;

    _rtdb = RtDB2Store::getInstance().getRtDB2(getRobotNumber(), getTeamChar());
}

cDecisionTree::~cDecisionTree()
{
    /* Chuck Norris made a Happy Meal cry. */
}

/*!
 * \brief Executes a single tree
 *
 * Determines the action for a given robotNr, based on the behavior tree and given mapParams.
 * When an action is found in the tree, mapParams is overwritten with the parameters to be used for the action.
 *
 * \param[in]       robotNr     The number of the robot to determine the action for.
 * \param[in]       behavior    The behavior defined by the role tree earlier.
 * \param[out]      action      The action to be determined from the behavior tree.
 * \param[inout]    mapParams   The parameters as input for the behavior tree. When the action is found, mapParams is overwritten with the params for the action.
 */
behTreeReturnEnum cDecisionTree::executeTree(const treeEnum& treeAsEnum, std::map<std::string, std::string> &mapParams, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes)
{
    try
    {
        std::string msg = "tree: ";
        msg.append( treeEnumToStr(treeAsEnum) );
        TRACE_FUNCTION(msg.c_str());
        //prettyPrintParams(mapParams);

        // Get the tree and its root node
        const cParsedTree& tree = getBehaviorTree(treeAsEnum);
        const cParsedNode& node = tree._nodeMapping.at(tree._rootNode);

        // Get the name of the tree and put into the diagnostics message
        std::string treeAsStr = treeEnumToStr(treeAsEnum);
        diagMsg.trees.push_back(treeAsStr);

        // This function can be called recursively.
        // Check the memoryStack.
        // If the stack is no longer valid (a different decision was made than last time):
        // 1) Remove the tree that is no longer chosen, and clear the memory of all nodes in this tree.
        // 2) Add the tree that is now chosen.

        // If I am the last element, add it to the stack. (Cold start)
        if (((int)(memoryStack.size()) - 1) < memoryStackIdx)
        {
            memoryStack.push_back( tree._id );
        }
        else
        {
            try
            {
                // If I am not the last element, see if my key corresponds to the index in the stack.

                // Check if the tree on the current level of the stack is similar to the tree we are currently traversing.
                if (memoryStack.at(memoryStackIdx) == tree._id)
                {
                    // Yes. We are retraversing a tree that was previously added to the stack.
                    // Do nothing.
                }
                else
                {
                    // No. We are actually traversing a different tree than the previous iteration.
                    // Remove all from stack up to and including the current stackIdx.
                    int iteration = 0;
                    while ((int)(memoryStack.size()) > memoryStackIdx)
                    {
                        iteration++;
                        // Find the tree that is to be removed
                        boost::uuids::uuid poppedTreeUuid = memoryStack.back();
                        treeEnum poppedTree = treeEnum::INVALID;
                        std::map<treeEnum, cParsedTree>::iterator itTree;
                        for (itTree = _trees.begin(); itTree != _trees.end(); ++itTree)
                        {
                            if (itTree->second._id == poppedTreeUuid)
                            {
                                poppedTree = itTree->first;
                            }
                        }

                        // Remove all memory of the popped tree
                        memoryStackNodes::iterator itNode;
                        for (itNode = memoryStackNodes.begin(); itNode != memoryStackNodes.end(); ) //intentionally empty)
                        {
                            if (itNode->first.first == poppedTree)
                            {
                                itNode = memoryStackNodes.erase(itNode);
                            }
                            else
                            {
                                ++itNode;
                            }
                        }

                        // Remove the tree from the stack
                        memoryStack.pop_back();
                        if (iteration > 100)
                        {
                            TRACE_WARNING("%d iterations in memory cleanup??", iteration);
                        }
                    }

                    // Finally, add the current traversing tree.
                    memoryStack.push_back(tree._id);
                }
            }
            catch (std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
                throw std::runtime_error(std::string("cDecisionTree::executeTree Linked to: ") + e.what());
            }
        }

        // Traverse the tree and execute the first action. (Or recursively call executeTree / traverseBehaviorTree )
        behTreeReturnEnum result = traverseBehaviorTree(tree, node, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);

        std::string treeStr = treeEnumToStr(treeAsEnum);
        std::string returnStr = behTreeReturnReverseMapping.at(result);
        TRACE("Tree ") << treeStr << " returning: " << returnStr;

        return result;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("Execute action failed. Linked to: ") + e.what());
    }
}

/*! \brief Loads the decision trees into memory.
 *
 *  Loads all .json files.
 *  Each .json can contain multiple trees, which are stored separately.
 */
void cDecisionTree::loadDecisionTrees(const std::string& directory)
{
    TRACE_FUNCTION("");
    try
    {
        std::string decisionTreePath = DECISIONTREE_PATH; // TODO what about argument 'directory'?

        // Iterate all *.json files
        boost::filesystem::path p(decisionTreePath);
        for(boost::filesystem::directory_iterator iDirectory(p); iDirectory != boost::filesystem::directory_iterator(); iDirectory++)
        {
            if(boost::filesystem::is_regular_file(iDirectory->path()))
            {
                // Check if it is a .json file
                std::string path = iDirectory->path().string();
                if ( path.substr(path.size() - 5).compare(".json") == 0 )
                {
                    // Parse the tree
                    boost::property_tree::ptree tree;
                    read_json(path, tree);

                    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, tree.get_child("trees"))
                    {
                        // Fetch tree enum from the tree's title
                        try
                        {
                            std::string strTitle = v.second.get<std::string>("title");
                            treeEnum tree = treeStrToEnum(strTitle);
                            // Store the tree
                            _trees[tree] = cParsedTree(v.second);
                        }
                        catch (std::exception &e)
                        {
                            std::string strTitle = v.second.get<std::string>("title");
                            TRACE_ERROR("Caught exception in file: %s while trying to convert %s", path.c_str(), strTitle.c_str());
                            throw std::runtime_error(std::string("Failed to load decision trees. Linked to: ") + e.what());
                        }
                    }
                }
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception while loading tree: %s", e.what());
        throw std::runtime_error(std::string("Failed to load decision trees. Linked to: ") + e.what());
    }
}

/*! \brief This function is for debugging purposes only.
 *
 *  This function prints the contents of the memory of the nodes / trees.
 *  It prints the stack of trees currently being run, and all nodes that are in memory with their return value.
 */
void cDecisionTree::prettyPrintMemoryStack(memoryStackType& memoryStack, memoryStackNodes& memoryStackNodes)
{
    memoryStackType::iterator itStack;
    int idx = 0;
    TRACE("memoryStack:");
    for (itStack = memoryStack.begin(); itStack != memoryStack.end(); ++itStack)
    {
        // iterate _trees, find tree with uuid == itStack
        std::map<treeEnum, cParsedTree>::iterator itTree;
        for (itTree = _trees.begin(); itTree != _trees.end(); ++itTree)
        {
            if (itTree->second._id == *itStack)
            {
                // x: "treeEnum"
                std::string tree = treeEnumToStr(itTree->first);
                TRACE("") << std::to_string(idx) << " " << tree;
                idx += 1;
            }
        }
    }

    TRACE("nodes in memory:");
    memoryStackNodes::iterator itNode;
    for (itNode = memoryStackNodes.begin(); itNode != memoryStackNodes.end(); ++itNode)
    {
        // <treeEnum, nodeName> : returnVal
        std::string tree = treeEnumToStr(itNode->first.first);

        const cParsedTree& nodeTreeObj = getBehaviorTree(itNode->first.first);
        const cParsedNode& nodeObj = nodeTreeObj._nodeMapping.at(itNode->first.second);
        std::string node = nodeObj._name;

        std::string returnVal = behTreeReturnReverseMapping.at(itNode->second);

        TRACE("<") << tree << ", " << node << "> : " << returnVal;
    }
}

// another debug function
void cDecisionTree::prettyPrintParams(std::map<std::string, std::string> const &mapParams)
{
    for (auto it = mapParams.begin(); it != mapParams.end(); ++it)
    {
        TRACE("param %s -> %s", it->first.c_str(), it->second.c_str());
    }
}

// We need the tree for finding new nodes.
// We need the 'current' node for recursion
behTreeReturnEnum cDecisionTree::traverseBehaviorTree(const cParsedTree& tree, const cParsedNode& node, std::map<std::string, std::string> &mapParams, const treeEnum& treeAsEnum, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes)
{
    try
    {
        std::string msg = "tree: ";
        msg.append( treeEnumToStr(treeAsEnum) );
        msg.append("; node: ");
        msg.append( node._name );
        TRACE_FUNCTION(msg.c_str());
        //prettyPrintParams(mapParams);

        // We are executing on this key (Tree, Node)
        memoryStackKey traversingKey = std::make_pair(treeAsEnum, node._id);

        if (memoryStackNodes.find(traversingKey) == memoryStackNodes.end())
        {
            // node does not exist in memory yet. Add to the memoryStack with INVALID return status.
            memoryStackNodes.insert( std::make_pair(traversingKey, behTreeReturnEnum::INVALID) );
        }

        // Turn this on to debug the memory of trees / nodes.
        //prettyPrintMemoryStack(memoryStack, memoryStackNodes);

        //// Add params for recursive calls.
        // 1. Make a copy of the node's properties, and all parameters passed down from the tree.
        // 2. Remove blacklisted parameters from the node's properties.
        // 3. Set "mapParams" to the node's properties.
        // 4. Add the parameters passed down from the tree. Existing parameters are ignored by std::map::insert.

        // 1. Make a copy of the node's properties, and all parameters passed down from the tree.
        std::map<std::string, std::string> mapPropertiesCopy = node._mapProperties;
        std::map<std::string, std::string> mapParamsCopy = mapParams;

        // These ignoredParameters will NOT be added to the list of parameters
        std::vector<std::string> ignoredParameters = {"wsf", "hasMemory", "parameter", "value"};

        // 2. Remove blacklisted parameters from the node's properties.
        // Remove blacklisted parameters (not to be added to mapParams) from copy
        std::vector<std::string>::iterator it;
        for (it = ignoredParameters.begin(); it != ignoredParameters.end(); ++it)
        {
            // Found blacklisted parameter, remove from mapPropertiesCopy
            if (mapPropertiesCopy.find(*it) != mapPropertiesCopy.end())
            {
                mapPropertiesCopy.erase(*it);
            }
        }
        // 3. Set "mapParams" to the node's properties.
        mapParams = mapPropertiesCopy;
        // 4. Add the parameters passed down from the tree. Existing parameters are ignored by std::map::insert.
        mapParams.insert(mapParamsCopy.begin(), mapParamsCopy.end());
        //// End code "Add params for recursive calls."

        switch(node._nodeType)
        {
            case nodeEnum::WORLDSTATE_FUNCTION:
            {
                robotNumber myRobotNr = getRobotNumber();

                TRACE("Entering WSF: ") << node._name;
                // Based on worldStateFunction that returns true/false, get node of corresponding result.
                if (!node._wsf(mapParams))
                {
                    std::string wsfmsg = node._name;
                    wsfmsg.append(": FALSE");
                    TRACE_SCOPE("WSF", wsfmsg.c_str());
                    // false
                    if (cWorldStateFunctions::getInstance().getRobotID() == myRobotNr) // Only trace my own WSF calls
                    {
                        TRACE("WSF: ") << node._name << ". FALSE";
                    }
                    const cParsedNode& childNode = tree._nodeMapping.at( node._children[0] );

                    behTreeReturnEnum nodeStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                    memoryStackNodes.at(traversingKey) = nodeStatus;
                    //memoryStack.at(memoryStackIdx).second.second = nodeStatus;
                    std::string returnStr = behTreeReturnReverseMapping.at(nodeStatus);
                    TRACE("WSF: ") << node._name << " returning " << returnStr;
                    return nodeStatus;
                }
                else
                {
                    std::string wsfmsg = node._name;
                    wsfmsg.append(": TRUE");
                    TRACE_SCOPE("WSF", wsfmsg.c_str());

                    // true
                    if (cWorldStateFunctions::getInstance().getRobotID() == myRobotNr) // Only trace my own WSF calls
                    {
                        TRACE("WSF: ") << node._name << ". TRUE";
                    }
                    const cParsedNode& childNode = tree._nodeMapping.at( node._children[1] );

                    behTreeReturnEnum nodeStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                    memoryStackNodes.at(traversingKey) = nodeStatus;
                    std::string returnStr = behTreeReturnReverseMapping.at(nodeStatus);
                    TRACE("WSF: ") << node._name << " returning " << returnStr;
                    return nodeStatus;
                }
                break;
            }

            case nodeEnum::MEMWORLDSTATE_FUNCTION:
            {
                robotNumber myRobotNr = getRobotNumber();

                // First check the current node's status.
                behTreeReturnEnum nodeStatus = memoryStackNodes.at(traversingKey);

                // If the worldStateFunction already passed or failed, return immediately.
                if (nodeStatus == behTreeReturnEnum::PASSED || nodeStatus == behTreeReturnEnum::FAILED)
                {
                    if (nodeStatus == behTreeReturnEnum::PASSED)
                    {
                        TRACE("MemWorldStateFunction returning PASSED");
                    }
                    else if (nodeStatus == behTreeReturnEnum::FAILED)
                    {
                        TRACE("MemWorldStateFunction returning FAILED");
                    }
                    return nodeStatus;
                }

                behTreeReturnEnum newChildStatus = behTreeReturnEnum::INVALID;

                // Based on worldStateFunction that returns true/false, get node of corresponding result.
                if (!node._wsf(mapParams))
                {
                    std::string wsfmsg = node._name;
                    wsfmsg.append(": FALSE");
                    TRACE_SCOPE("MEMWSF", wsfmsg.c_str());

                    // false
                    if (cWorldStateFunctions::getInstance().getRobotID() == myRobotNr) // Only trace my own WSF calls
                    {
                        TRACE("WSF: ") << node._name << ". FALSE";
                    }

                    const cParsedNode& childNode = tree._nodeMapping.at( node._children[0] );
                    newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                }
                else
                {
                    std::string wsfmsg = node._name;
                    wsfmsg.append(": TRUE");
                    TRACE_SCOPE("MEMWSF", wsfmsg.c_str());

                    // true
                    if (cWorldStateFunctions::getInstance().getRobotID() == myRobotNr) // Only trace my own WSF calls
                    {
                        TRACE("WSF: ") << node._name << ". TRUE";
                    }
                    const cParsedNode& childNode = tree._nodeMapping.at( node._children[1] );
                    newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                }

                if (newChildStatus == behTreeReturnEnum::FAILED)
                {
                    // Set worldStateFunction to FAILED
                    memoryStackNodes.at(traversingKey) = behTreeReturnEnum::FAILED;
                    TRACE("MemWorldStateFunction returning FAILED");
                    return behTreeReturnEnum::FAILED;
                }
                else if (newChildStatus == behTreeReturnEnum::PASSED)
                {
                    // Set worldStateFunction to PASSED
                    memoryStackNodes.at(traversingKey) = behTreeReturnEnum::PASSED;
                    TRACE("MemWorldStateFunction returning PASSED");
                    return behTreeReturnEnum::PASSED;
                }
                else if (newChildStatus == behTreeReturnEnum::RUNNING)
                {
                    memoryStackNodes.at(traversingKey) = behTreeReturnEnum::RUNNING;
                    TRACE("MemWorldStateFunction returning RUNNING");
                    return behTreeReturnEnum::RUNNING;
                }
                else
                {
                    memoryStackNodes.at(traversingKey) = behTreeReturnEnum::INVALID;
                    TRACE("MemWorldStateFunction returning INVALID");
                    return behTreeReturnEnum::INVALID;
                }

                break;
            }

            case nodeEnum::RULE:
            {
                // TODO - implement. Evaluate rule to true/false.
                TRACE_ERROR("traverseBehaviorTree TODO rule not implemented");
                break;
            }

            case nodeEnum::INVERTER:
            {
                // TODO - implement.
                TRACE_ERROR("TODO inverter not implemented");
                break;
            }

            case nodeEnum::ROLE:
            {
                TRACE_SCOPE("ROLE", node._name.c_str());
                TRACE("Found Role: ") << node._name;

                // Store the role of the current robot's ID, new style:
                teamplay::robotStore::getInstance().setOwnRobotRole(node._role);

                // Old style:
                cWorldStateFunctions::getInstance().setRobotRole(cWorldStateFunctions::getInstance().getRobotID(), node._role);

                // If this is not our own robot ID, return without traversing further.
                robotNumber myRobotNr = getRobotNumber();
                if (cWorldStateFunctions::getInstance().getRobotID() == myRobotNr)
                {
                    // Execute the role's tree recursively.
//                    behTreeReturnEnum nodeStatus = executeTree(node._role, mapParams, memoryStack, (memoryStackIdx+1), memoryStackNodes);
//                    memoryStackNodes.at(traversingKey) = nodeStatus;
//                    std::string returnStr = behTreeReturnReverseMapping.at(nodeStatus);
//                    TRACE("Role ") << node._name << " returning " << returnStr;
//                    return nodeStatus;
                    return behTreeReturnEnum::INVALID;
                }
                else
                {
                    // We are executing this tree to obtain this robot's role. Return.
                    return behTreeReturnEnum::INVALID;
                }


                break;
            }

            case nodeEnum::BEHAVIOR:
            {
                TRACE_SCOPE("BEHAVIOR", node._name.c_str());
                TRACE("Found Behavior: ") << node._name;

                // Execute the behavior's tree recursively.
                behTreeReturnEnum nodeStatus = executeTree(node._behavior, mapParams, memoryStack, (memoryStackIdx+1), memoryStackNodes);
                memoryStackNodes.at(traversingKey) = nodeStatus;

                std::string returnStr = behTreeReturnReverseMapping.at(nodeStatus);
                TRACE("Behavior ") << node._name << " returning " << returnStr;
                return nodeStatus;

                break;
            }

            case nodeEnum::ACTION:
            {
                // Execute action, store node status, and return.
                if(behTreeReturnEnum::INVALID == memoryStackNodes.at(traversingKey))
                {
                    TRACE("Entering Action: ") << node._name;
                }

                behTreeReturnEnum nodeStatus = executeAction(node, mapParams);
                memoryStackNodes.at(traversingKey) = nodeStatus;

                TRACE("Action: ") << node._name << " returning " << behTreeReturnReverseMapping.at(nodeStatus);

                return nodeStatus;

                break;
            }

            case nodeEnum::MEMSEQUENCE:
            {
                // Parse the sequence node (As part of BehaviorTree)
                // AKA the "AND" node

                /*
                1 for i from 1 to n do
                2     childstatus <- Tick(child(i))
                3     if childstatus = running
                4        return running
                5     else if childstatus = failure
                6        return failure
                7 end
                8 return success
                */

                TRACE("Entering MemSequence");

                // First check the current node's status.
                behTreeReturnEnum nodeStatus = memoryStackNodes.at(traversingKey);

                // If the sequence already passed or failed, return immediately.
                if (nodeStatus == behTreeReturnEnum::PASSED)
                {
                    TRACE("MemSequence returning PASSED (from memory)");
                    return nodeStatus;
                }
                else if (nodeStatus == behTreeReturnEnum::FAILED)
                {
                    TRACE("MemSequence returning FAILED (from memory)");
                    return nodeStatus;
                }

                // Check the memory of its children.

                BOOST_FOREACH(const boost::uuids::uuid &id, node._children)
                {
                    // Only traverse the child if it has not passed or failed.
                    const cParsedNode& childNode = tree._nodeMapping.at(id);

                    memoryStackKey childKey = std::make_pair(treeAsEnum, childNode._id);

                    // First make sure the child exists in the memoryStack under this sequence's children.
                    if (memoryStackNodes.find(childKey) == memoryStackNodes.end())
                    {
                        // child does not exist yet. Add to the memoryStack with INVALID return status.
                        memoryStackNodes.insert( std::make_pair(childKey, behTreeReturnEnum::INVALID) );
                    }

                    // Get the child's status as stored in the memoryStack
                    behTreeReturnEnum childStatus = memoryStackNodes.at(childKey);

                    // If the child has passed/failed before, skip this child.
                    if (childStatus == behTreeReturnEnum::PASSED || childStatus == behTreeReturnEnum::FAILED)
                    {
                        continue;
                    }

                    // Child has not passed or failed. Traverse.
                    behTreeReturnEnum newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                    // Store the child's status in the memoryStack.
                    memoryStackNodes.at(childKey) = newChildStatus;

                    if (newChildStatus == behTreeReturnEnum::RUNNING)
                    {
                        // Set sequence to running
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::RUNNING;
                        TRACE("MemSequence returning RUNNING");
                        return behTreeReturnEnum::RUNNING;
                    }
                    else if (newChildStatus == behTreeReturnEnum::FAILED)
                    {
                        // Set sequence to failed
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::FAILED;
                        TRACE("MemSequence returning FAILED");
                        return behTreeReturnEnum::FAILED;
                    }
                    // else if (childStatus == behTreeReturnEnum::PASSED) { continue; }
                }

                // If this point is reached, all children have passed.
                // Set sequence to passed.
                memoryStackNodes.at(traversingKey) = behTreeReturnEnum::PASSED;
                TRACE("MemSequence returning PASSED");
                return behTreeReturnEnum::PASSED;

                break;
            }

            case nodeEnum::SEQUENCE:
            {
                // Parse the sequence node (As part of BehaviorTree)
                // AKA the "AND" node

                /*
                1 for i from 1 to n do
                2     childstatus <- Tick(child(i))
                3     if childstatus = running
                4        return running
                5     else if childstatus = failure
                6        return failure
                7 end
                8 return success
                */

                TRACE("Entering Sequence");

                BOOST_FOREACH(const boost::uuids::uuid &id, node._children)
                {
                    const cParsedNode& childNode = tree._nodeMapping.at(id);

                    // Child has not passed or failed. Traverse.
                    behTreeReturnEnum newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);

                    if (newChildStatus == behTreeReturnEnum::RUNNING)
                    {
                        // Set sequence to running
                        TRACE("Sequence returning RUNNING");
                        return behTreeReturnEnum::RUNNING;
                    }
                    else if (newChildStatus == behTreeReturnEnum::FAILED)
                    {
                        // Set sequence to failed
                        TRACE("Sequence returning FAILED");
                        return behTreeReturnEnum::FAILED;
                    }
                    // else if (childStatus == behTreeReturnEnum::PASSED) { continue; }
                }

                // If this point is reached, all children have passed.
                // Set sequence to passed.
                TRACE("Sequence returning PASSED");
                return behTreeReturnEnum::PASSED;

                break;
            }

            case nodeEnum::MEMSELECTOR:
            {
                // Parse the selector node (As part of BehaviorTree)
                // AKA the "OR" node

                /*
                1 for i from 1 to n do
                2     childstatus <- Tick(child(i))
                3     if childstatus = running
                4        return running
                5     else if childstatus = success
                6        return success
                7 end
                8 return failure
                */

                TRACE("Entering MemSelector");

                // First check the current node's status.
                behTreeReturnEnum nodeStatus = memoryStackNodes.at(traversingKey);

                // If the selector already passed or failed, return immediately.
                if (nodeStatus == behTreeReturnEnum::PASSED)
                {
                    TRACE("MemSequence returning PASSED (from memory)");
                    return nodeStatus;
                }
                else if (nodeStatus == behTreeReturnEnum::FAILED)
                {
                    TRACE("MemSequence returning FAILED (from memory)");
                    return nodeStatus;
                }

                // Check the memory of its children.

                BOOST_FOREACH(const boost::uuids::uuid &id, node._children)
                {

                    // Only traverse the child if it has not passed or failed.
                    const cParsedNode& childNode = tree._nodeMapping.at(id);

                    memoryStackKey childKey = std::make_pair(treeAsEnum, childNode._id);

                    // First make sure the child exists in the memoryStack under this sequence's children.
                    if (memoryStackNodes.find(childKey) == memoryStackNodes.end())
                    {
                        // child does not exist yet. Add to the memoryStack with INVALID return status.
                        memoryStackNodes.insert( std::make_pair(childKey, behTreeReturnEnum::INVALID) );
                    }

                    // Get the child's status as stored in the memoryStack
                    behTreeReturnEnum childStatus = memoryStackNodes.at(childKey);

                    // If the child has passed/failed before, skip this child.
                    if (childStatus == behTreeReturnEnum::PASSED || childStatus == behTreeReturnEnum::FAILED)
                    {
                        continue;
                    }

                    // Child has not passed or failed. Traverse.
                    behTreeReturnEnum newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);
                    // Store the child's status in the memoryStack.
                    memoryStackNodes.at(childKey) = newChildStatus;

                    if (newChildStatus == behTreeReturnEnum::RUNNING)
                    {
                        // Set selector to running
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::RUNNING;
                        TRACE("MemSelector returning RUNNING");
                        return behTreeReturnEnum::RUNNING;
                    }
                    else if (newChildStatus == behTreeReturnEnum::PASSED)
                    {
                        // Set selector to passed
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::PASSED;
                        TRACE("MemSelector returning PASSED");
                        return behTreeReturnEnum::PASSED;
                    }
                    // else if (childStatus == behTreeReturnEnum::FAILED) { continue; }
                }

                // If this point is reached, all children have failed.
                // Set selector to failed.
                memoryStackNodes.at(traversingKey) = behTreeReturnEnum::FAILED;
                TRACE("MemSelector returning FAILED");
                return behTreeReturnEnum::FAILED;

                break;
            }

            case nodeEnum::SELECTOR:
            {
                // Parse the selector node (As part of BehaviorTree)
                // AKA the "OR" node

                /*
                1 for i from 1 to n do
                2     childstatus <- Tick(child(i))
                3     if childstatus = running
                4        return running
                5     else if childstatus = success
                6        return success
                7 end
                8 return failure
                */

                TRACE("Entering Selector");

                BOOST_FOREACH(const boost::uuids::uuid &id, node._children)
                {
                    // Get the child
                    const cParsedNode& childNode = tree._nodeMapping.at(id);

                    // Traverse.
                    behTreeReturnEnum newChildStatus = traverseBehaviorTree(tree, childNode, mapParams, treeAsEnum, memoryStack, memoryStackIdx, memoryStackNodes);

                    if (newChildStatus == behTreeReturnEnum::RUNNING)
                    {
                        // Set selector to running
                        TRACE("Selector returning RUNNING");
                        return behTreeReturnEnum::RUNNING;
                    }
                    else if (newChildStatus == behTreeReturnEnum::PASSED)
                    {
                        // Set selector to passed
                        TRACE("Selector returning PASSED");
                        return behTreeReturnEnum::PASSED;
                    }
                    // else if (childStatus == behTreeReturnEnum::FAILED) { continue; }
                }

                // If this point is reached, all children have failed.
                // Set selector to failed.
                TRACE("Selector returning FAILED");
                return behTreeReturnEnum::FAILED;

                break;
            }

            case nodeEnum::PARAMETER_CHECK:
            {
                // Checks parameters that have been set while traversing trees.
                // Returns PASSED if the "parameter" matches the given "value".
                // Returns FAILED otherwise.

                try
                {

                    // Get "parameter" name, e.g., areaOfInterest
                    std::string paramName = node._mapProperties.at("parameter");
                    // Use it as key in the mapping from role, e.g., "areaOfInterest": "A_OPP_SIDE"
                    std::string mapParamVal = mapParams.at(paramName);
                    // Check mapParamVal with the node's "value"
                    if (mapParamVal.compare(node._mapProperties.at("value")) == 0)
                    {
                        TRACE("ParameterCheck: ") << node._mapProperties.at("parameter") << " Value: " << node._mapProperties.at("value") << " PASSED.";
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::PASSED;
                        return behTreeReturnEnum::PASSED;
                    }
                    else
                    {
                        TRACE("ParameterCheck: ") << node._mapProperties.at("parameter") << " Value: " << node._mapProperties.at("value") << " FAILED.";
                        memoryStackNodes.at(traversingKey) = behTreeReturnEnum::FAILED;
                        return behTreeReturnEnum::FAILED;
                    }

                }
                catch (std::exception &e)
                {
                    TRACE_ERROR("Caught exception: %s - Does the parameter exist? parameter: %s", e.what(), node._mapProperties.at("parameter").c_str());
                    throw std::runtime_error(std::string("cDecisionTree::traverseBehaviorTree Linked to: ") + e.what());
                }

                break;
            }

            default:
            {
                TRACE("Unexpected nodeType received: ") << node._name;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cDecisionTree::traverseBehaviorTree Linked to: ") + e.what());
    }
    return behTreeReturnEnum::FAILED;
}


const cParsedTree& cDecisionTree::getBehaviorTree(const treeEnum &behavior)
{
    try
    {
        return _trees.at(behavior);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Failed to find tree for behavior %s", enum2str(behavior));
        throw std::runtime_error(std::string("Failed to find tree for behavior ") + enum2str(behavior));
    }
}

behTreeReturnEnum cDecisionTree::executeAction(const cParsedNode &node, std::map<std::string, std::string> &mapParams)
{
    TRACE_FUNCTION(node._name.c_str());
    // Get the name of the action and put into the diagnostics message
    diagMsg.trees.push_back(node._name);

    // Copy the names of nodes 1, 2, 3 and N for easier diagnostics
    if (diagMsg.trees.size() >= 4)
    {
        diagMsg.state = diagMsg.trees.front();
        diagMsg.role = diagMsg.trees.at(1);
        diagMsg.behavior = diagMsg.trees.at(2);
        diagMsg.action = diagMsg.trees.back();
    }

    // Copy the contents of the diagnostics store into the diagnostics message
    diagMsg.aiming = diagnosticsStore::getDiagnostics().isAiming();
    auto shootTarget = diagnosticsStore::getDiagnostics().getShootTarget();
    diagMsg.shootTargetX = shootTarget.x;
    diagMsg.shootTargetY = shootTarget.y;

    // Publish the role for the avoid-duplicate-role functionality
    cRTDBOutputAdapter::getInstance().setRole(diagMsg.role);

    // Send diagnostics message and clear the diagnostics store for the next iteration
    if (_rtdb != NULL)
    {
        _rtdb->put(DIAG_TEAMPLAY, &diagMsg); // TODO: move to diagnostics store? But, it would not be a store anymore .. diagnosticsAdapter then?
    }
    diagnosticsStore::getInstance().clear();

    //// Set the node's parameters to mapParams.
    // 1. Make a copy of the node's properties, and all parameters passed down from the tree.
    std::map<std::string, std::string> mapPropertiesCopy = node._mapProperties;
    std::map<std::string, std::string> mapParamsCopy = mapParams;
    // 2. Set "mapParams" to the node's properties.
    mapParams = mapPropertiesCopy;
    // 3. Add the parameters passed down from the tree. Existing parameters are ignored by std::map::insert.
    mapParams.insert(mapParamsCopy.begin(), mapParamsCopy.end());
    //// End "Set the node's parameters to mapParams."

    // Trace all parameters passed to the action
    std::stringstream str;
    str << "Parameters: {";
    std::map<std::string, std::string>::const_iterator it;
    for (it = mapParams.begin(); it != mapParams.end(); ++it)
    {
        str << "'" << it->first << "': '" << it->second << "', ";
    }
    str << "}";
    TRACE(str.str().c_str());

    return node._action->execute(mapParams);
}

treeEnum treeStrToEnum(const std::string enumString)
{
    try
    {
        return treeEnumMapping.at(enumString);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("Failed to parse tree name '" + enumString + "' to treeEnum. Linked to: " + e.what()));
    }
    return treeEnum::INVALID;
}

std::string treeEnumToStr(const treeEnum treeEnumVal)
{
    try
    {
        std::map<std::string, treeEnum>::const_iterator it;
        for(it = treeEnumMapping.begin(); it != treeEnumMapping.end(); ++it)
        {
            if(it->second == treeEnumVal)
            {
                return it->first;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("Failed to parse treeEnum '" + std::to_string((int)treeEnumVal) + "' to std::string. Linked to: " + e.what()));
    }
    return "INVALID";
}

void cDecisionTree::clearDiagMsg()
{
    TRACE_FUNCTION("");
    diagMsg.trees.clear();
}
