 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * peripheralsAdapterBallPossessionROS.hpp
 *
 *  Created on: Oct 7, 2016
 *      Author: Tim Kouters
 */

#ifndef PERIPHERALSADAPTERBALLPOSSESSIONROS_HPP_
#define PERIPHERALSADAPTERBALLPOSSESSIONROS_HPP_

#include <boost/shared_ptr.hpp>
#include <vector>

#include "ros/node_handle.h"
#include "ros/service.h"

#include "worldModel/claim_own_ball_possession.h"
#include "worldModel/release_own_ball_possession.h"

#include "int/facilities/identifierGenerator.hpp"
#include "int/facilities/templatedSubject.hpp"


class peripheralsAdapterBallPossessionROS : public templatedSubject<bool>
{
	public:
		peripheralsAdapterBallPossessionROS();
		~peripheralsAdapterBallPossessionROS();

		void InitializeROS();
	private:
		boost::shared_ptr<ros::NodeHandle> _hROS;
        ros::ServiceServer _srvClaimBallPossession;
        ros::ServiceServer _srvReleaseBallPossession;


        bool claim_own_ball_possession_cb(
        		worldModel::claim_own_ball_possession::Request &req,
        		worldModel::claim_own_ball_possession::Response &resp);

        bool release_own_ball_possession_cb(
        		worldModel::release_own_ball_possession::Request &req,
        		worldModel::release_own_ball_possession::Response &resp);
};

#endif /* PERIPHERALSADAPTERBALLPOSSESSIONROS_HPP_ */
