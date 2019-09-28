 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuration.hpp
 *
 */

#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

namespace motionPlanning
{

class configuration {
public:
    configuration();
    virtual ~configuration();

    virtual void setGetBall_ObstacleThreshold(const float obstacleThreshold);
    virtual float getGetBall_ObstacleThreshold() const;
    virtual void setGetBall_BallSpeedThreshold(const float ballSpeedThreshold);
    virtual float getGetBall_BallSpeedThreshold() const;
    virtual void setGetBall_BallSpeedScaling(const float ballSpeedScaling);
    virtual float getGetBall_BallSpeedScaling() const;

    virtual void setInterceptBall_ObstacleThreshold(const float obstacleThreshold);
    virtual float getInterceptBall_ObstacleThreshold() const;
    virtual void setInterceptBall_BallSpeedThreshold(const float ballSpeedThreshold);
    virtual float getInterceptBall_BallSpeedThreshold() const;
    virtual void setInterceptBall_BallSpeedScaling(const float ballSpeedScaling);
    virtual float getInterceptBall_BallSpeedScaling() const;

    virtual void setKeeperMove_YMaxOffset(const float yMaxOffset);
    virtual float getKeeperMove_YMaxOffset() const;
    virtual void setKeeperMove_XGoalpostOffset(const float xGoalpostOffset);
    virtual float getKeeperMove_XGoalpostOffset() const;
    virtual void setKeeperMove_XYThreshold(const float xyThreshold);
    virtual float getKeeperMove_XYThreshold() const;

    virtual void setMoveToTarget_XYThreshold(const float xyThreshold);
    virtual float getMoveToTarget_XYThreshold() const;
    virtual void setMoveToTarget_PhiThreshold(const float phiThreshold);
    virtual float getMoveToTarget_PhiThreshold() const;

    virtual void setPassToTarget_Accuracy(const float accuracy);
    virtual float getPassToTarget_Accuracy() const;
    virtual void setPassToTarget_Timeout(const float timeout);
    virtual float getPassToTarget_Timeout() const;

    virtual void setShootAtTarget_AimSettleTime(const float aimSettleTime);
    virtual float getShootAtTarget_AimSettleTime() const;
    virtual void setShootAtTarget_Accuracy(const float accuracy);
    virtual float getShootAtTarget_Accuracy() const;
    virtual void setShootAtTarget_Timeout(const float timeout);
    virtual float getShootAtTarget_Timeout() const;
    virtual void setShootAtTarget_CoarseAngle(const float coarseAngle);
    virtual float getShootAtTarget_CoarseAngle() const;
    virtual void setShootAtTarget_DisableBHDelay(const float disableBHDelay);
    virtual float getShootAtTarget_DisableBHDelay() const;
    virtual void setShootAtTarget_SleepAfterShoot(const float sleepAfterShoot);
    virtual float getShootAtTarget_SleepAfterShoot() const;

    virtual void setTurnAwayFromOpponent_PhiThreshold(const float phiThreshold);
    virtual float getTurnAwayFromOpponent_PhiThreshold() const;

private:
   float _getBall_obstacleThreshold;
   float _getBall_ballSpeedThreshold;
   float _getBall_ballSpeedScaling;

   float _interceptBall_obstacleThreshold;
   float _interceptBall_ballSpeedThreshold;
   float _interceptBall_ballSpeedScaling;

   float _keeperMove_YMaxOffset;
   float _keeperMove_XGoalpostOffset;
   float _keeperMove_XYThreshold;

   float _moveToTarget_XYThreshold;
   float _moveToTarget_PhiThreshold;

   float _passToTarget_accuracy;
   float _passToTarget_timeout;

   float _shootAtTarget_aimSettleTime;
   float _shootAtTarget_accuracy;
   float _shootAtTarget_timeout;
   float _shootAtTarget_coarseAngle;
   float _shootAtTarget_disableBhDelay;
   float _shootAtTarget_sleepAfterShoot;

   float _turnAwayFromOpponent_PhiThreshold;
};

}/* namespace motionplanning */

#endif /* CONFIGURATION_HPP_ */
