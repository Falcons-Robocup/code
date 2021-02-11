// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDecisionTree.hpp
 *
 *  Created on: Apr 24, 2016
 *      Author: Erik Kouters
 */

#ifndef CDECISIONTREE_HPP_
#define CDECISIONTREE_HPP_

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include "cDiagnostics.hpp"
#include "FalconsRtDB2.hpp"

#include "int/types/cDecisionTreeTypes.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/types/cActionMapping.hpp"

#include "int/actions/cAbstractAction.hpp"
#include "int/types/cWorldStateFunctionTypes.hpp"

// memoryStack example:
// Tree = T1, node = {A, ..., Z}
// A = sequence with children B and C.
// <T1, A> : {<T1, B>=PASSED, <T1, C>=RUNNING}, RUNNING
// <T1, C> : RUNNING


typedef std::pair<treeEnum,boost::uuids::uuid> memoryStackKey;
typedef std::map< memoryStackKey, behTreeReturnEnum > memoryStackNodes; // to remember the nodes
//typedef std::pair< memoryStackChildren, behTreeReturnEnum > memoryStackValue;
//typedef std::pair< memoryStackKey, memoryStackValue > memoryStackEntry;
typedef std::vector< boost::uuids::uuid > memoryStackType;


class cParsedNode
{
    public:
        cParsedNode() {
            _behavior = treeEnum::INVALID;
            _role = treeEnum::INVALID;
            _nodeType = nodeEnum::INVALID;
        };
        cParsedNode(const boost::property_tree::ptree &root);
        ~cParsedNode() { };

        boost::property_tree::ptree _root;
        boost::uuids::uuid _id;
        nodeEnum _nodeType;
        std::string _name;
        std::string _title;
        std::map<std::string, std::string> _mapProperties;
        std::vector<boost::uuids::uuid> _children;

        boost::shared_ptr<cAbstractAction> _action;
        treeEnum _behavior;
        treeEnum _role;
        worldstateFunctionType _wsf;
};

class cParsedTree
{
    public:
        cParsedTree() { };
        cParsedTree(const boost::property_tree::ptree &root);
        ~cParsedTree() { };

        boost::property_tree::ptree _root;
        boost::uuids::uuid _rootNode;
        boost::uuids::uuid _id;
        std::map<boost::uuids::uuid, cParsedNode> _nodeMapping;
};

class cDecisionTree
{
	public:
        static cDecisionTree& getInstance()
        {
            static cDecisionTree instance; // Guaranteed to be destroyed.
                                          // Instantiated on first use.
            return instance;
        }

		void loadDecisionTrees(const std::string& directory);
        behTreeReturnEnum executeTree(const treeEnum& tree, std::map<std::string, std::string> &mapParams, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes);
        void clearDiagMsg();
        
        diagTeamplay diagMsg;

	private:
        cDecisionTree();
        ~cDecisionTree();
        cDecisionTree(cDecisionTree const&); // Don't Implement
        void operator=(cDecisionTree const&); // Don't implement

		const std::string DECISIONTREE_PATH = pathToTeamplayDataRepo();

        RtDB2 *_rtdb = NULL;
		// These are the trees that are loaded on startup.
		std::map<treeEnum, cParsedTree> _trees;

        behTreeReturnEnum traverseBehaviorTree(const cParsedTree& tree, const cParsedNode& node, std::map<std::string, std::string> &mapParams, const treeEnum& treeAsEnum, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes);
        behTreeReturnEnum executeAction(const cParsedNode &node, std::map<std::string, std::string> &mapParams);
        const cParsedTree& getBehaviorTree(const treeEnum &behavior);
        void prettyPrintMemoryStack(memoryStackType& memoryStack, memoryStackNodes& memoryStackNodes);
        void prettyPrintParams(std::map<std::string, std::string> const &mapParams);

};

#endif /* CDECISIONTREE_HPP_ */
