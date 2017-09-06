 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTeamplayControlInterface.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Jan Feitsma
 */

#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "int/cTeamplayControlInterface.hpp"
#include "int/types/cDecisionTreeTypes.hpp"
#include "int/utilities/trace.hpp"

cTeamplayControlInterface::cTeamplayControlInterface()
{
    TRACE("constructing cTeamplayControlInterface");
    reset();
}

void cTeamplayControlInterface::getOverrideState(cOverrideState &overrideState)
{
    overrideState = _overrideState;
}

void cTeamplayControlInterface::reset()
{
    _overrideState.active = false;
    _overrideState.level = overrideLevelEnum::INVALID;
    _overrideState.gameState = treeEnum::INVALID;
    _overrideState.role = treeEnum::INVALID;
    _overrideState.behavior = treeEnum::INVALID;
    _overrideState.action = actionEnum::INVALID;
    _overrideState.params.clear();
}
std::string cTeamplayControlInterface::parse(std::string command)
{
	std::string myResultString="INVALID COMMAND";
    boost::algorithm::trim(command);
    TRACE("parse: ") << command;
    reset();
    try
    {
        std::vector<std::string> words;
        boost::split(words, command, boost::is_any_of("\t "));
        TRACE("number of words: ") << std::to_string(words.size());
        if( words.size()==0)
        {
    	    TRACE_ERROR("invalid commandType, no input string found");
    	    return("PARSE ERROR: NO INPUT STRING");
        }

        std::string commandType = words[0];

        if( words.size()==1 )
        {
		   if ( commandType == "reset" )
           {
               reset();
               myResultString="RESET DONE";
           }
		   else if ( commandType == "disabled" )
		   {
		       _overrideState.level = overrideLevelEnum::DISABLED;
		       _overrideState.active = true;
		       myResultString="TEAMPLAY DISABLED";
		   }
           else if (commandType == "getActionList")
           {
           	std::stringstream ss;
   			TRACE("command received: getActionList");
   			for( auto iterator = actionMapping.begin(); iterator != actionMapping.end(); iterator++)
   			{
   				ss << iterator->first << ";" ;
   			}

   			myResultString= ss.str();
           }
           else if (commandType == "getShootMappingList")
           {
           	std::stringstream ss;
   			TRACE("command received: getShootMappingList");
   			for( auto iterator = shootMapping.begin(); iterator != shootMapping.end(); iterator++)
   			{
   				ss << iterator->first << ";" ;
   			}

   			myResultString= ss.str();
           }
           else if (commandType == "getBehaviorList")
           {
           	std::stringstream ss;
   			TRACE("command received: getBehaviorList");
   			for( auto iterator = treeEnumMapping.begin(); iterator != treeEnumMapping.end(); iterator++)
   			{
   				ss << iterator->first << ";" ;
   			}
   			myResultString= ss.str();
           }
           else if (commandType == "getRoleList")
           {
           	std::stringstream ss;
   			TRACE("command received: getRoleList");
   			for( auto iterator = treeEnumMapping.begin(); iterator != treeEnumMapping.end(); iterator++)
   			{
   				ss << iterator->first << ";" ;
   			}
   			myResultString= ss.str();
           }
           else if (commandType == "getGameStateList")
           {
           	std::stringstream ss;
   			TRACE("command received: getGameStateList");
   			for( auto iterator = treeEnumMapping.begin(); iterator != treeEnumMapping.end(); iterator++)
   			{
   				ss << iterator->first << ";" ;
   			}
   			myResultString= ss.str();
           }
        }
        else
        {
            // require at least one extra word
        	std::string param1 = words[1];
        	if ( commandType == "getActionParamList" )
        	{
              	std::stringstream ss;
       			TRACE("command received: getActionParamList");
       			{
       				std::string actionString=param1;
       				actionEnum theAction=actionMapping[actionString];
        			boost::shared_ptr<cAbstractAction> actionObjectPtr = enumToActionMapping.at( theAction );
    				for( auto paramIterator = actionObjectPtr->_actionParameters.begin(); paramIterator != actionObjectPtr->_actionParameters.end(); paramIterator++)
    				{
    	   				ss << paramIterator->first << ";" ;
    				}
       			}
       			myResultString= ss.str();
        	}
        	else
        	{
               std::string enumValueAsStr = words[1];
               // determine parameters
               for (int i = 2; i < (int)words.size(); ++i)
               {
                   std::vector<std::string> keyval;
                   boost::split(keyval, words[i], boost::is_any_of("="));
                   if (keyval.size() == 2)
                   {
                       TRACE("parameter: ") << keyval[0] << " has value: " << keyval[1];
                       _overrideState.params[keyval[0]] = keyval[1];
                   }
               }
               // switch
               if (commandType == "gamestate")
               {
                   _overrideState.gameState = treeEnumMapping[enumValueAsStr];
                   _overrideState.level = overrideLevelEnum::GAMESTATE;
               }
               else if (commandType == "role")
               {
                   _overrideState.role = treeEnumMapping[enumValueAsStr];
                   _overrideState.level = overrideLevelEnum::ROLE;
               }
               else if (commandType == "behavior")
               {
                   _overrideState.behavior = treeEnumMapping[enumValueAsStr];
                   _overrideState.level = overrideLevelEnum::BEHAVIOR;
               }
               else if (commandType == "action")
               {
                   _overrideState.action = actionMapping[enumValueAsStr];
                   _overrideState.level = overrideLevelEnum::ACTION;

    		    	TRACE("overriding action");

    			   boost::shared_ptr<cAbstractAction> actionObjectPtr = enumToActionMapping.at(_overrideState.action);
    			   behTreeReturnEnum myResult=actionObjectPtr->execute(_overrideState.params);
    			   myResultString = behTreeReturnReverseMapping[myResult];
               }
               else
               {
        	       TRACE_ERROR("invalid commandType: ") << commandType;
        	       myResultString="PARSE ERROR";
               }

               _overrideState.active = true;
               //TRACE_INFO_TIMEOUT(5.0, "TP control override: %s", command.c_str());
               // TODO fix this; without timeout it would generate spam when for instance intercepting, hence timeout
        	}
        }
        return myResultString.c_str();
    } 
	catch (std::exception &e)
    {
        reset();
	    TRACE_ERROR("Caught exception: ") << e.what();
	    return("ERROR");
    }

    
}


