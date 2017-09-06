 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTeamplayControlInterface.hpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Jan Feitsma
 */

#ifndef CTEAMPLAYCONTROLINTERFACE_HPP_
#define CTEAMPLAYCONTROLINTERFACE_HPP_


#include <vector>
#include <string>
#include "int/types/cDecisionTreeTypes.hpp"
#include "int/types/cActionTypes.hpp"

enum class overrideLevelEnum
{
    INVALID,
    GAMESTATE,
    ROLE,
    BEHAVIOR,
    ACTION,
    DISABLED
};

 
struct cOverrideState
{
    bool                     active;
    overrideLevelEnum        level;
    treeEnum            gameState;
    treeEnum             behavior;
    treeEnum                 role;
    actionEnum               action;
    std::map< std::string, std::string> params;
};

class cTeamplayControlInterface
{
    public:
		static cTeamplayControlInterface& getInstance()
		{
			static cTeamplayControlInterface instance;
			return instance;
		}
		void getOverrideState(cOverrideState &overrideState);
		std::string parse(std::string command);

	private:
		cTeamplayControlInterface();
		void reset();
		cOverrideState _overrideState;
};

#endif /* CTEAMPLAYCONTROLINTERFACE_HPP_ */
