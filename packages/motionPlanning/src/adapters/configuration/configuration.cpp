 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuration.cpp
 *
 */

#include "int/types/configuration.hpp"

using namespace motionPlanning;

configuration::configuration()
{}

configuration::~configuration()
{}

void configuration::setGetBall_ObstacleThreshold(const float obstacleThreshold)
{
	_getBall_obstacleThreshold = obstacleThreshold;
}
float configuration::getGetBall_ObstacleThreshold() const
{
	return _getBall_obstacleThreshold;
}
void configuration::setGetBall_BallSpeedThreshold(const float ballSpeedThreshold)
{
   _getBall_ballSpeedThreshold = ballSpeedThreshold;
}
float configuration::getGetBall_BallSpeedThreshold() const
{
	return _getBall_ballSpeedThreshold;
}
void configuration::setGetBall_BallSpeedScaling(const float ballSpeedScaling)
{
   _getBall_ballSpeedScaling = ballSpeedScaling;
}
float configuration::getGetBall_BallSpeedScaling() const
{
	return _getBall_ballSpeedScaling;
}

void configuration::setInterceptBall_ObstacleThreshold(const float obstacleThreshold)
{
   _interceptBall_obstacleThreshold = obstacleThreshold;
}
float configuration::getInterceptBall_ObstacleThreshold() const
{
	return _interceptBall_obstacleThreshold;
}
void configuration::setInterceptBall_BallSpeedThreshold(const float ballSpeedThreshold)
{
   _interceptBall_ballSpeedThreshold = ballSpeedThreshold;
}
float configuration::getInterceptBall_BallSpeedThreshold() const
{
	return _interceptBall_ballSpeedThreshold;
}
void configuration::setInterceptBall_BallSpeedScaling(const float ballSpeedScaling)
{
   _interceptBall_ballSpeedScaling = ballSpeedScaling;
}
float configuration::getInterceptBall_BallSpeedScaling() const
{
	return _interceptBall_ballSpeedScaling;
}

void configuration::setKeeperMove_YMaxOffset(const float yMaxOffset)
{
   _keeperMove_YMaxOffset = yMaxOffset;
}
float configuration::getKeeperMove_YMaxOffset() const
{
	return _keeperMove_YMaxOffset;
}
void configuration::setKeeperMove_XGoalpostOffset(const float xGoalpostOffset)
{
   _keeperMove_XGoalpostOffset = xGoalpostOffset;
}
float configuration::getKeeperMove_XGoalpostOffset() const
{
	return _keeperMove_XGoalpostOffset;
}
void configuration::setKeeperMove_XYThreshold(const float xyThreshold)
{
   _keeperMove_XYThreshold = xyThreshold;
}
float configuration::getKeeperMove_XYThreshold() const
{
	return _keeperMove_XYThreshold;
}

void configuration::setMoveToTarget_XYThreshold(const float xyThreshold)
{
   _moveToTarget_XYThreshold = xyThreshold;
}
float configuration::getMoveToTarget_XYThreshold() const
{
	return _moveToTarget_XYThreshold;
}
void configuration::setMoveToTarget_PhiThreshold(const float phiThreshold)
{
   _moveToTarget_PhiThreshold = phiThreshold;
}
float configuration::getMoveToTarget_PhiThreshold() const
{
	return _moveToTarget_PhiThreshold;
}

void configuration::setPassToTarget_Accuracy(const float accuracy)
{
   _passToTarget_accuracy = accuracy;
}
float configuration::getPassToTarget_Accuracy() const
{
	return _passToTarget_accuracy;
}
void configuration::setPassToTarget_Timeout(const float timeout)
{
   _passToTarget_timeout = timeout;
}
float configuration::getPassToTarget_Timeout() const
{
	return _passToTarget_timeout;
}

void configuration::setShootAtTarget_AimSettleTime(const float aimSettleTime)
{
   _shootAtTarget_aimSettleTime = aimSettleTime;
}
float configuration::getShootAtTarget_AimSettleTime() const
{
	return _shootAtTarget_aimSettleTime;
}
void configuration::setShootAtTarget_Accuracy(const float accuracy)
{
   _shootAtTarget_accuracy = accuracy;
}
float configuration::getShootAtTarget_Accuracy() const
{
	return _shootAtTarget_accuracy;
}
void configuration::setShootAtTarget_Timeout(const float timeout)
{
   _shootAtTarget_timeout = timeout;
}
float configuration::getShootAtTarget_Timeout() const
{
	return _shootAtTarget_timeout;
}
void configuration::setShootAtTarget_CoarseAngle(const float coarseAngle)
{
   _shootAtTarget_coarseAngle = coarseAngle;
}
float configuration::getShootAtTarget_CoarseAngle() const
{
	return _shootAtTarget_coarseAngle;
}
void configuration::setShootAtTarget_DisableBHDelay(const float disableBHDelay)
{
   _shootAtTarget_disableBhDelay = disableBHDelay;
}
float configuration::getShootAtTarget_DisableBHDelay() const
{
	return _shootAtTarget_disableBhDelay;
}
void configuration::setShootAtTarget_SleepAfterShoot(const float sleepAfterShoot)
{
   _shootAtTarget_sleepAfterShoot = sleepAfterShoot;
}
float configuration::getShootAtTarget_SleepAfterShoot() const
{
	return _shootAtTarget_sleepAfterShoot;
}

void configuration::setTurnAwayFromOpponent_PhiThreshold(const float phiThreshold)
{
   _turnAwayFromOpponent_PhiThreshold = phiThreshold;
}
float configuration::getTurnAwayFromOpponent_PhiThreshold() const
{
	return _turnAwayFromOpponent_PhiThreshold;
}
