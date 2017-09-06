 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterWorldModel.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/cRosAdapterWorldModel.hpp"

#include <iostream>
#include <stdexcept>
#include <algorithm>

#include <cDiagnosticsEvents.hpp>
#include <FalconsCommon.h>
#include <WorldModelNames.h>

using std::exception;
using std::runtime_error;
using std::endl;
using std::cerr;

cRosAdapterWorldModel::cRosAdapterWorldModel(PeripheralsInterfaceData &piData) :
	_piData(piData) {
	TRACE(">");

	_previousBallPossession = false;

	TRACE("<");
}

cRosAdapterWorldModel::~cRosAdapterWorldModel() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterWorldModel::initialize() {
	TRACE(">");

	bool connected = ros::service::waitForService(
			WorldModelInterface::s_set_own_encoder_displacement,
			ros::Duration(5));

	if (connected) {
		ros::service::waitForService(
				WorldModelInterface::s_claim_own_ball_possession);
		ros::service::waitForService(
				WorldModelInterface::s_release_own_ball_possession);


		_srvSetDisplacement = _n.serviceClient<
				worldModel::set_own_encoder_displacement>(
				WorldModelInterface::s_set_own_encoder_displacement, true);
		_srvClaimBallPossession = _n.serviceClient<
				worldModel::claim_own_ball_possession>(
				WorldModelInterface::s_claim_own_ball_possession, true);
		_srvReleaseBallPossession = _n.serviceClient<
				worldModel::release_own_ball_possession>(
				WorldModelInterface::s_release_own_ball_possession, true);

//		_subWmInfo = _n.subscribe(WorldModelInterface::t_wmInfo, 1, &cRosAdapterWorldModel::worldModel_cb, this);
	} else {
		TRACE_ERROR("Could not initialize peripheralsInterface WorldModel subscription!");
		cerr << "ERROR   : Could not initialize peripheralsInterface WorldModel subscription!" << endl;
	}

	TRACE("<");
}

void cRosAdapterWorldModel::notifyWorldModel() {
	TRACE(">");

	worldModel::set_own_encoder_displacement srv;

	piVelAcc currentVelocity = _piData.getVelocityOutput();
	piDisplacement currentDisplacement = _piData.getDisplacementOutput();

        srv.request.dx = currentDisplacement.pos.getX();
	srv.request.dy = currentDisplacement.pos.getY();
	srv.request.dtheta = project_angle_mpi_pi(currentDisplacement.pos.getPhi());

        srv.request.vx = currentVelocity.vel.x;
	srv.request.vy = currentVelocity.vel.y;
	srv.request.vtheta = currentVelocity.vel.phi;

	if( (fabs(srv.request.dx) < 0.2) &&
		(fabs(srv.request.dy) < 0.2) &&
		(fabs(srv.request.dtheta) < 0.2))
	{
		if(!_srvSetDisplacement.call(srv))
		{
			initialize();
			TRACE_ERROR("failed to send displacement to worldModel");
		}

	}
        else
        {
                TRACE_ERROR("Ignored spike in peripheral encoders towards WorldModel");
        }

	// Claim / Release ball possession
	bool hasBall = 	_piData.getHasBall();

	// Determine if we need to claim / release ball possession
	if (!_previousBallPossession && hasBall) {
		// Previously did not have ball, but now we do. Claim ball possession.

		worldModel::claim_own_ball_possession srvClaim;

		if (!_srvClaimBallPossession.call(srvClaim)) {
			initialize();
			//TRACE_ERROR("Failed to claim ball possession");
		}
	} else if (_previousBallPossession && !hasBall) {
		// Previously we did have the ball, but now we do not anymore. Release ball possession.

		worldModel::release_own_ball_possession srvRelease;

		if (!_srvReleaseBallPossession.call(srvRelease)) {
			initialize();
			//TRACE_ERROR("Failed to release ball possession");
		}
	}
	_previousBallPossession = hasBall;

	TRACE("< hasBall=%d", (int)hasBall);
}

//void cRosAdapterWorldModel::worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg) {
//	TRACE(">");
//	bool isActive = false;
//
//	if (msg->robotStatus.status_type == rosMsgs::s_robot_status::TYPE_INPLAY)
//	{
//		isActive = true;
//	}
//
//	_piData.setRobotActive(isActive);
//	TRACE("isActive = %d", isActive);
//	TRACE("<");
//}
