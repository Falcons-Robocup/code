 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * worldModelInfoUpdaterROS.hpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#ifndef WORLDMODELINFOUPDATERROS_HPP_
#define WORLDMODELINFOUPDATERROS_HPP_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "ros/node_handle.h"
#include "ros/publisher.h"

#include "cDiagnostics.hpp"
#include "worldModel/t_wmInfo.h"
#include "rosMsgs/t_diag_wm_top.h"

#include "int/types/ball/ballPossessionType.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "int/types/robot/robotStatusType.hpp"
#include "int/types/robot/robotType.hpp"


class worldModelInfoUpdaterROS
{
	public:
		worldModelInfoUpdaterROS();
		~worldModelInfoUpdaterROS();

		void InitializeROS();

		void setBallLocation(ballClass_t ball);
		void setBallPossession(ballPossessionClass_t possession);
		void setObstacles(std::vector<obstacleClass_t> obstacles);
		void setRobotLocation(robotClass_t robotLocation);
		void setRobotStatus(robotStatusType status);
		void setActiveRobots(std::vector<uint8_t> activeRobots);
		void setTeamMembers(std::vector<robotClass_t> members);
		void sendPacket();

	private:
		boost::shared_ptr<ros::NodeHandle> _hROS;
        ros::Publisher  _pWmInfo;
        worldModel::t_wmInfo _msgWmInfo;
        diagnostics::cDiagnosticsSender<rosMsgs::t_diag_wm_top> *_diagSender = NULL;
        rosMsgs::t_diag_wm_top _diagMsg;

        void sendDiagnostics();
};

#endif /* WORLDMODELINFOUPDATERROS_HPP_ */
