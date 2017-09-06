 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include "rosMsgs/t_diag_teamplay.h"
#include "cDiagnostics.hpp"

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

        behTreeReturnEnum executeTree(const treeEnum& tree, std::map<std::string, std::string> &mapParams, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes);
        void clearDiagMsg();


	private:
        cDecisionTree();
        ~cDecisionTree();
        cDecisionTree(cDecisionTree const&); // Don't Implement
        void operator=(cDecisionTree const&); // Don't implement

		const std::string DECISIONTREE_PATH = "/falcons/code/packages/teamplay/teamplayData/";

		// These are the trees that are loaded on startup.
		std::map<treeEnum, cParsedTree> _trees;

		// Diagnostics
		diagnostics::cDiagnosticsSender<rosMsgs::t_diag_teamplay> diagSender;
		rosMsgs::t_diag_teamplay diagMsg;

		void loadDecisionTrees();
        behTreeReturnEnum traverseBehaviorTree(const cParsedTree& tree, const cParsedNode& node, std::map<std::string, std::string> &mapParams, const treeEnum& treeAsEnum, memoryStackType& memoryStack, int memoryStackIdx, memoryStackNodes& memoryStackNodes);
        behTreeReturnEnum executeAction(const cParsedNode &node);
        const cParsedTree& getBehaviorTree(const treeEnum &behavior);
        void prettyPrintMemoryStack(memoryStackType& memoryStack, memoryStackNodes& memoryStackNodes);

};

#endif /* CDECISIONTREE_HPP_ */
