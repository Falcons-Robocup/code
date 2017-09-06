 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorMixedTeamPacketROS.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#ifndef CONFIGURATORMIXEDTEAMPACKETROS_HPP_
#define CONFIGURATORMIXEDTEAMPACKETROS_HPP_

#include <boost/function.hpp>
#include <dynamic_reconfigure/server.h>

#include "worldModelSync/mixedteamConfig.h"

class configuratorMixedTeamPacketROS
{
	public:
		configuratorMixedTeamPacketROS();
		~configuratorMixedTeamPacketROS();

		void loadDefaultParameters();
		void initializeROS();

		void addNotifyNewConfigFunction(boost::function<void()> func)
		{
		  _fncNotifyNewConfigurations.push_back(func);
		}

	private:
		std::vector<boost::function<void()>> _fncNotifyNewConfigurations;
		boost::shared_ptr<ros::NodeHandle> _hROS;
		boost::shared_ptr<dynamic_reconfigure::Server<worldModelSync::mixedteamConfig>> _srvConfig;
		dynamic_reconfigure::Server<worldModelSync::mixedteamConfig>::CallbackType _srvDynConfig_cb;

		void reconfig_cb(worldModelSync::mixedteamConfig &config, uint32_t level);
};

#endif /* CONFIGURATORMIXEDTEAMPACKETROS_HPP_ */
