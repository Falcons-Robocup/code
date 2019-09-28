 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTokyoDrift.cpp
 *
 *  Created on: 2017-03-02
 *      Author: Erik Kouters
 */

#include <boost/algorithm/string.hpp>
#include <sstream>
#include <fstream>
#include <math.h>

#include "int/algorithms/cTokyoDriftVelocity.hpp"
#include "int/algorithms/cPhiPID.hpp"

//#define RADIUS 2.0 // meters -- radius between robot center and ball center.
//#define FACING_TARGET_TOLERANCE 0.5 // radians (0.5rad == ~28deg) -- the tolerance when the robot is sufficiently looking at the target.
//#define MAX_STEP_ANGLE (M_PI / 16)
//#define TOKYO_DRIFT_VPHI 2.5 // radians per sec -- the velocity to perform tokyo drift with

static int signFunction(double x)
{
    if (x > 0.0)
    {
        return 1;
    }
    else if (x < 0.0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// R = Robot position
// B = Ball position
// T = Target position
// S = Subtarget position
// While doing tokyo drift, we compute intermediate subtargets to ensure we turn around the ball.
// For example, when Tokyo Drifting for 180 degrees, we just turn around if we compute no intermediate setpoints.
// This function computes a subtarget that turns the robot around the ball with a fixed angle.
void cTokyoDriftVelocity::calculateTokyoDriftSubtarget(const Position2D& R, const Position2D& B, const Position2D& T, const double& targetPhi, Position2D& S)
{
    Position2D robotPos = R;
    Position2D ballPos = B;
    Vector2D targetPosVec = T.xy();
    Vector2D ballPosVec = B.xy();



    // deltaPhi is the angle between robot facing and looking towards target
    double deltaPhi = project_angle_mpi_pi(targetPhi - robotPos.phi);

    // Determine direction of turning
    int direction = signFunction(deltaPhi);

    TRACE("targetPhi=%12.9f ; deltaPhi=%12.9f ; direction=%d", targetPhi, deltaPhi, direction);

    // Compute RT -- The final target after Tokyo Drifting around the ball.
    Vector2D robotTargetPosVec = ( ( (ballPosVec - targetPosVec).size() + _tokyoDriftParams.radius ) / (ballPosVec - targetPosVec).size() ) * (ballPosVec - targetPosVec) + targetPosVec;
    Position2D robotTargetPos = Position2D(robotTargetPosVec.x, robotTargetPosVec.y, targetPhi);

    TRACE("robotTargetPos.x=%12.9f ; robotTargetPos.y=%12.9f ; robotTargetPos.phi=%12.9f", robotTargetPos.x, robotTargetPos.y, robotTargetPos.phi);

    // Set orientation of ball to face RT
    ballPos.phi = angle_between_two_points_0_2pi(ballPos.x, ballPos.y, robotTargetPos.x, robotTargetPos.y);

    TRACE("ballPos.phi=%12.9f", ballPos.phi);

    // Transform robotPos FCS2RCS on ballPos. (origin is now on ballPos)
    robotPos.transform_fcs2rcs(ballPos);

    TRACE("After transform: robotPos.x=%12.9f ; robotPos.y=%12.9f ; robotPos.phi=%12.9f", robotPos.x, robotPos.y, robotPos.phi);

    // First compute alphaR :: the angle between B and R.
    double alphaR = atan2(robotPos.y, robotPos.x);

    TRACE("alphaR=%12.9f", alphaR);

    // Compute the angle between B and RP
    double alphaRP = 0.0;
    if (deltaPhi < _tokyoDriftParams.step_angle)
    {
        alphaRP = alphaR + ((double)direction * deltaPhi);
    }
    else
    {
        alphaRP = alphaR + ((double)direction * _tokyoDriftParams.step_angle);
    }

    TRACE("alphaRP=%12.9f", alphaRP);

    // Compute S and have him look at origin (ballPos) -- RP is the subtarget step for doing the tokyo drift.
    S.x = _tokyoDriftParams.radius * cos(alphaRP);
    S.y = _tokyoDriftParams.radius * sin(alphaRP);
    S.phi = angle_between_two_points_0_2pi(S.x, S.y, 0.0, 0.0);

    TRACE("Before transform: S.x=%12.9f ; S.y=%12.9f ; S.phi=%12.9f", S.x, S.y, S.phi);

    // Convert RP back to FCS
    S.transform_rcs2fcs(ballPos);
}

/* Overwrite functionality of cAbstractPathPlanning update function */
void cTokyoDriftVelocity::execute()
{
    TRACE_FUNCTION(_ppData.pos.tostr());
    TRACE("> target: %s", _ppData.pos.tostr());
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    pp_plot_data_struct_t plotdata;
    _main->_ppData->getPlotData(plotdata);

    // Get obstacles from cPathPlanningData
    //std::vector<pp_obstacle_struct_t> obstacles;
    //_main->_ppData->getObstacles(obstacles);

    // Get HaveBall from cPathPlanningData
    bool haveBall = false;
    _main->_ppData->getHaveBall(haveBall);

    // Get tokyo drift params
    _main->_ppData->getTokyoDriftParams(_tokyoDriftParams);

    // Get BallPos from cPathPlanningData
    //Position2D ballPos;
    //_main->_ppData->getBallPos(ballPos);
    // Set ballPos as position at RADIUS distance from robot in facing direction.
    Position2D ballPos = Position2D(0.0, _tokyoDriftParams.radius, 0.0);
    ballPos.transform_rcs2fcs(currPos);

    /*
     * Set tokyo drift as subtarget facing the original target.
     *
     * Two situations -- from a large distance to target, and when close to target (e.g., turn only).
     */

//    _ppData.pos.x = 1;
//    _ppData.pos.y = 3;
//    _ppData.pos.phi = 0;
//
//    currPos.x = 4;
//    currPos.y = 2;
//    currPos.phi = 1.67;
//
//    ballPos.x = 4;
//    ballPos.y = 2.2;
//
//    haveBall = true;

//    if (haveBall && (calc_distance(currPos, _ppData.pos) > 4.0*_tokyoDriftParams.radius) )
//    {
//        // This is the long distance situation.
//        /*
//         * On long distance, we are only allowed to drive towards the target "forward", keeping the ball in front of us.
//         * We have a "threshold" when the robot is facing the target sufficiently before it's allowed to drive forward.
//         */
//        double targetPhi = atan2(_ppData.pos.y - currPos.y, _ppData.pos.x - currPos.x);
//        double deltaPhi = project_angle_mpi_pi(targetPhi - currPos.phi);
//
//        if (fabs(deltaPhi) > _tokyoDriftParams.facing_target_tol)
//        {
//            // Not allowed to drive forward, compute subtarget turning around the ball.
//
//            // targetPhi is the angle between R and T -- We want to face the target (x,y) position.
//            //double targetPhi = atan2(_ppData.pos.y - currPos.y, _ppData.pos.x - currPos.x);
//
//            //Position2D S;
//            //calculateTokyoDriftSubtarget(currPos, ballPos, _ppData.pos, targetPhi, S);
////
////            // Determine direction of turning
////            int direction = signFunction(deltaPhi);
////
////            TRACE("direction=%d", direction);
////
////            // Given: R, T, B
////            Position2D robotPos = currPos;
////            Vector2D targetPosVec = _ppData.pos.xy();
////            Vector2D ballPosVec = ballPos.xy();
////
////            // Compute RT
////            Vector2D robotTargetPosVec = ( ( (ballPosVec - targetPosVec).size() + RADIUS ) / (ballPosVec - targetPosVec).size() ) * (ballPosVec - targetPosVec) + targetPosVec;
////            Position2D robotTargetPos = Position2D(robotTargetPosVec.x, robotTargetPosVec.y, targetPhi);
////
////            TRACE("robotTargetPos.x=%12.9f ; robotTargetPos.y=%12.9f ; robotTargetPos.phi=%12.9f", robotTargetPos.x, robotTargetPos.y, robotTargetPos.phi);
////
////            // Set orientation of ball to face RT
////            ballPos.phi = angle_between_two_points_0_2pi(ballPos.x, ballPos.y, robotTargetPos.x, robotTargetPos.y);
////
////            TRACE("ballPos.phi=%12.9f", ballPos.phi);
////
////            // Transform robotPos FCS2RCS on ballPos. (origin is now on ballPos)
////            robotPos.transform_fcs2rcs(ballPos);
////
////            TRACE("After transform: robotPos.x=%12.9f ; robotPos.y=%12.9f ; robotPos.phi=%12.9f", robotPos.x, robotPos.y, robotPos.phi);
////
////            // First compute alphaR :: the angle between B and R.
////            double alphaR = atan2(robotPos.y, robotPos.x);
////
////            TRACE("alphaR=%12.9f", alphaR);
////
////            // Compute the angle between B and RP
////            double alphaRP = alphaR + ((double)direction * MAX_STEP_ANGLE);
////
////            TRACE("alphaRP=%12.9f", alphaRP);
////
////            // Compute RP and have him look at origin (ballPos)
////            Position2D robotPPos;
////            robotPPos.x = RADIUS * cos(alphaRP);
////            robotPPos.y = RADIUS * sin(alphaRP);
////            robotPPos.phi = angle_between_two_points_0_2pi(robotPPos.x, robotPPos.y, 0.0, 0.0);
////
////            TRACE("Before transform: robotPPos.x=%12.9f ; robotPPos.y=%12.9f ; robotPPos.phi=%12.9f", robotPPos.x, robotPPos.y, robotPPos.phi);
////
////            // Convert RP back to FCS
////            robotPPos.transform_rcs2fcs(ballPos);
//
//            //TRACE("After transform: robotPPos.x=%12.9f ; robotPPos.y=%12.9f ; robotPPos.phi=%12.9f", S.x, S.y, S.phi);
//
//            //_ppData.pos.x = S.x;
//            //_ppData.pos.y = S.y;
//            //_ppData.pos.phi = S.phi;
//
//            // The new target is standing behind the ball, recompute the vPhi using PID.
////            _ppData.pos.phi = targetPhi;
////            cAbstractPathPlanning* phiPid = new cPhiPID(this->_main);
////            phiPid->computeDt();
////            phiPid->setData(_ppData);
////            phiPid->execute();
////            _ppData = phiPid->getData();
////            delete phiPid;
//
//            // vx = vphi * radius
//            _ppData.vel.y = 0.0;
//            _ppData.vel.x = _ppData.vel.phi * _tokyoDriftParams.radius;
//
//            //TRACE("overriding subtarget for tokyo drift: x=%12.9f, y=%12.9f, phi=%12.9f", _ppData.pos.x, _ppData.pos.y, _ppData.pos.phi);
//            TRACE("overriding long range velocity for tokyo drift: x=%12.9f, y=%12.9f, phi=%12.9f", _ppData.vel.x, _ppData.vel.y, _ppData.vel.phi);
//        }
//        else
//        {
//            // Facing the target sufficiently. Drive towards the target.
//
//            //Vector2D sourceVec = ballPos.xy();
//            //Vector2D destVec = _ppData.pos.xy();
//
//            //Vector2D targetVec = (1 - (RADIUS / (destVec - sourceVec).size())) * (destVec - sourceVec) + sourceVec;
//            //_ppData.pos.x = targetVec.x;
//            //_ppData.pos.y = targetVec.y;
//            //_ppData.pos.phi = targetPhi;
//
//            // The new target is standing behind the ball, recompute the vPhi using PID.
////            _ppData.pos.phi = targetPhi;
////            cAbstractPathPlanning* phiPid = new cPhiPID(this->_main);
////            phiPid->computeDt();
////            phiPid->setData(_ppData);
////            phiPid->execute();
////            _ppData = phiPid->getData();
////            delete phiPid;
//
//            //_ppData.vel.phi = 0.0;
//            TRACE("already looking towards target. Drive forward.");
//        }
//    }
    if (haveBall)
    {
        // Short distance situation
        // close to target, turn around ball towards target facing if facing_tolerance is not met.

        double deltaPhi = project_angle_mpi_pi(_ppData.pos.phi - currPos.phi);

        if (fabs(deltaPhi) > _tokyoDriftParams.facing_target_tol)
        {
            // vx = vphi * radius
            _ppData.vel.y = 0.0;
            _ppData.vel.x = _ppData.vel.phi * _tokyoDriftParams.radius;

            TRACE("overriding close range velocity for tokyo drift: x=%12.9f, y=%12.9f, phi=%12.9f", _ppData.vel.x, _ppData.vel.y, _ppData.vel.phi);
            plotdata.vel_t.x = _ppData.vel.x;
            plotdata.tokyo_drift = 1;
        }
        else
        {
            TRACE("not overriding close range velocity: x=%12.9f, y=%12.9f, phi=%12.9f", _ppData.vel.x, _ppData.vel.y, _ppData.vel.phi);
            plotdata.tokyo_drift = 0;
        }
    }

    _main->_ppData->setPlotData(plotdata);


    /*
     * Tokyo Drift overwrites some of the default behavior.
     * This means this algorithm is expected to be performed _LAST_.
     *
     * 1. As first check, if we have the ball and further than 50cm from target.
     *
     * 2. Secondly, we must be facing the target (not at the angle the target wants us to have).
     * If this does not hold, override velocity (vx, vy, vphi) to do Tokyo Drift.
     *
     * 3. We now face the (sub)target.
     * <s>Override phi of the (sub)target to have the same facing.</s>
     * Nope. Set vphi to 0. Drive directly towards the (sub)target.
     */

    // Check #1: If haveBall && dist(robot, target) > 50cm
//    if (haveBall && (calc_distance(currPos, _ppData.pos) > 0.5) )
//    {
//        double targetPhi = atan2(_ppData.pos.y - currPos.y, _ppData.pos.x - currPos.x);
//
//        double deltaPhi = targetPhi - currPos.phi;
//        while (deltaPhi >= M_PI)
//        {
//            deltaPhi -= 2*M_PI;
//        }
//        while (deltaPhi <= -M_PI)
//        {
//            deltaPhi += 2*M_PI;
//        }
//
//        // Check #2: Robot phi is facing target within tolerance FACING_TARGET_TOLERANCE
//        if (fabs(deltaPhi) > FACING_TARGET_TOLERANCE)
//        {
//            // Determine the sign of deltaPhi
//            double direction = 0.0;
//            if (deltaPhi > 0.0)
//            {
//                direction = 1.0;
//            }
//            else if (deltaPhi < 0.0)
//            {
//                direction = -1.0;
//            }
//            else
//            {
//                direction = 0.0;
//            }
//
//            double vTmp = TOKYO_DRIFT_VPHI * direction;
//
//            _ppData.vel.x = vTmp * RADIUS;
//            _ppData.vel.y = 0.0;
//            _ppData.vel.phi = vTmp;
//
//            TRACE("overriding velocity for tokyo drift: vx=%12.9f, vy=%12.9f, vphi=%12.9f", _ppData.vel.x, _ppData.vel.y, _ppData.vel.phi);
//        }
//        else
//        {
//            //Check #3: Set vphi to 0. Drive directly towards the (sub)target.
//            _ppData.vel.phi = 0.0;
//            TRACE("already looking towards target. overriding vphi to 0.");
//        }
//    }

    //Position2D subTarget = _ppData.pos;

    // Publish subtarget to diagnostics
    //_main->_ppData->publishSubtarget(subTarget.x, subTarget.y);
    
    //_ppData.pos.x = subTarget.x;
    //_ppData.pos.y = subTarget.y;

}
