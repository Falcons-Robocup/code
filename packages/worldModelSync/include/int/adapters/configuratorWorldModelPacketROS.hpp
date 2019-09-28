 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorWorldModelPacketROS.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#ifndef CONFIGURATORWORLDMODELPACKETROS_HPP_
#define CONFIGURATORWORLDMODELPACKETROS_HPP_

#include <boost/function.hpp>
#include <vector>
#include <dynamic_reconfigure/server.h>

#include "worldModelSync/worldmodelsyncConfig.h"

class configuratorWorldModelPacketROS
{
	public:
		configuratorWorldModelPacketROS();
		~configuratorWorldModelPacketROS();

		void loadDefaultParameters();
		void initializeROS();

		void addNotifyNewConfigFunction(boost::function<void()> func)
		{
		  _fncNotifyNewConfigurations.push_back(func);
		}

	private:
		std::vector<boost::function<void()>> _fncNotifyNewConfigurations;
		boost::shared_ptr<ros::NodeHandle> _hROS;
		dynamic_reconfigure::Server<worldModelSync::worldmodelsyncConfig> _srvDynConfig;
		dynamic_reconfigure::Server<worldModelSync::worldmodelsyncConfig>::CallbackType _srvDynConfig_cb;

		void reconfig_cb(worldModelSync::worldmodelsyncConfig &config, uint32_t level);
};

#endif /* CONFIGURATORWORLDMODELPACKETROS_HPP_ */
