 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningMain.cpp
 *
 *  Created on: Sept 06, 2015
 *      Author: Erik Kouters
 */

#include "int/cPathPlanningMain.hpp"

#include <boost/thread/thread.hpp>

#include "cDiagnosticsDutyCycle.hpp"

#include "int/algorithms/cXYLinear.hpp"
#include "int/algorithms/cPhiLinear.hpp"
#include "int/algorithms/cXYPID.hpp"
#include "int/algorithms/cXPID.hpp"
#include "int/algorithms/cYPID.hpp"
#include "int/algorithms/cPhiPID.hpp"
#include "int/algorithms/cXYPFM.hpp"
#include "int/algorithms/cPhiPFM.hpp"
#include "int/algorithms/cXYTrajectory.hpp"
#include "int/algorithms/cXYLinearAcceleration.hpp"
#include "int/algorithms/cPhiLinearAcceleration.hpp"
#include "int/algorithms/cXYTurnThenMove.hpp"
#include "int/algorithms/cPhiMoveThenTurn.hpp"
#include "int/algorithms/cMoveAtSpeed.hpp"
#include "int/algorithms/cXYBrake.hpp"
#include "int/algorithms/cTokyoDrift.hpp"
#include "int/algorithms/cTokyoDriftVelocity.hpp"
#include "int/algorithms/cFCStoRCS.hpp"
#include "int/algorithms/cLimitVelocities.hpp"
#include "int/algorithms/cBoundaryLimiter.hpp"

#define EXPECTED_FREQUENCY (30.0) // TODO store somewhere common, provide by heartBeat? simulator uses 10Hz, real robot 30Hz
#define WARNING_THRESHOLD (0.6)
diagnostics::cDiagnosticsDutyCycle dutyCycleObserver("pathPlanning", EXPECTED_FREQUENCY, WARNING_THRESHOLD);

cPathPlanningMain::cPathPlanningMain()
{
    // Empty implementation. Defined to allow this class to be globally defined.

    _prev_vel = Velocity2D(0.0, 0.0, 0.0);
    _algorithm = NULL;
    _ppData = NULL;
    _isCurrentLowestActiveRobot = false;
    _obstacleAvoidanceCurrentlyEnabled = true;
}

//
//cPathPlanningMain::cPathPlanningMain(cPathPlanningData& data)
//{
//    _ppData = data;
//    _algorithm = NULL;
//    _dt = 1.0 / cPathPlanningMain::UPDATE_FREQUENCY;
//
//    /*
//    // Set callback to ReconfigureAdapter
//    CallbackType callback;
//    callback = boost::bind(&cPathPlanningMain::setAlgorithms, this, _1);
//    _rcAdapter.setCallback(callback);
//    _tpAdapter.setCallback(callback);
//
//    _rcAdapter.initializeRC();
//    _wmAdapter.initializeWM();
//    _tpAdapter.initializeTP();
//    */
//}

cPathPlanningMain::~cPathPlanningMain()
{
}

void cPathPlanningMain::iterate()
{
    while (true)
    {
        TRACE(">");
        try
        {
            // Check WorldModel and Teamplay active
            bool wmActive;
            bool tpActive;
            bool isLowestActiveRobot;
            bool obstacleAvoidanceIsEnabled;
            _ppData->getRobotActive(wmActive);
            _ppData->getPathPlanningActivated(tpActive);
            _ppData->getIsLowestActiveRobot(isLowestActiveRobot);
            _ppData->getObstacleAvoidanceIsEnabled(obstacleAvoidanceIsEnabled);

            if (wmActive && tpActive)
            {
                dutyCycleObserver.pokeStart();

                // Check if _algorithm needs to be updated
                pp_algorithm_type newAlgo;
                _ppData->getAlgorithmType(newAlgo);
                if ((newAlgo != _currAlgType) ||
                	(_algorithm == NULL) ||
                	(_isCurrentLowestActiveRobot != isLowestActiveRobot) ||
                	(_obstacleAvoidanceCurrentlyEnabled != obstacleAvoidanceIsEnabled)
                   )
                {
                    setAlgorithms(newAlgo, isLowestActiveRobot, obstacleAvoidanceIsEnabled);
                }

                // Do an execute of the algorithm
                if (_algorithm != NULL)
                {
                    _algorithm->execute();
                }

                dutyCycleObserver.pokeEnd();
            }
            else
            {
                std::stringstream ss;
                ss << "PathPlanning is running, but refuses to move. RobotInPlay(WM): " << wmActive << " ; PathPlanningActivated(TP): " << tpActive;
                TRACE(ss.str().c_str());
            }

            // Sleep
            boost::this_thread::sleep_for( boost::chrono::seconds(1800) );
            TRACE("Thread waking up... THIS MAY NOT HAPPEN!");
        }
        catch(boost::thread_interrupted& e)
        {
            // Thread interrupted.
            //TRACE("Thread interrupted!");
        }
        catch(...)
        {
            TRACE("Unknown exception caught!");
        }
        TRACE("<");
    }
}

void cPathPlanningMain::setAlgorithms(const pp_algorithm_type &pp_algoType, const bool isLowestActiveRobot, const bool obstacleAvoidanceEnabled)
{
    TRACE("Setting algorithms...");

    // Only set new algorithm if it is a different type (or if it is still NULL)
    if ((pp_algoType != _currAlgType) || (_algorithm == NULL) ||
    		(_isCurrentLowestActiveRobot != isLowestActiveRobot) || (_obstacleAvoidanceCurrentlyEnabled != obstacleAvoidanceEnabled))
    {

        if (_algorithm != NULL)
        {
            // Delete all entries, then delete the algorithm
            while(!_algorithm->_ppBlocks.empty()) delete _algorithm->_ppBlocks.front(), _algorithm->_ppBlocks.pop_front();
            delete _algorithm;
        }

        _isCurrentLowestActiveRobot = isLowestActiveRobot;
        _obstacleAvoidanceCurrentlyEnabled = obstacleAvoidanceEnabled;

        switch(pp_algoType)
        {

            case moveWhileTurning:
            {
                _algorithm = new cAbstractPathPlanning(this);

                //_algorithm->_ppBlocks.push_back(new cTokyoDrift(this));

                /* Ignore obstacle avoidance for keeper robot */
                if ((!_isCurrentLowestActiveRobot) || (obstacleAvoidanceEnabled))
                {
                    _algorithm->_ppBlocks.push_back(new cXYTrajectory(this));
                }

                _algorithm->_ppBlocks.push_back(new cBoundaryLimiter(this));
                _algorithm->_ppBlocks.push_back(new cTokyoDrift(this));
                _algorithm->_ppBlocks.push_back(new cXYPID(this));
                //_algorithm->_ppBlocks.push_back(new cXYBrake(this));
                _algorithm->_ppBlocks.push_back(new cPhiPID(this));
                _algorithm->_ppBlocks.push_back(new cLimitVelocities(this));
                _algorithm->_ppBlocks.push_back(new cFCStoRCS(this));
                _algorithm->_ppBlocks.push_back(new cTokyoDriftVelocity(this));
                //_algorithm->_ppBlocks.push_back(new cTokyoDrift(this)); // Do not use -- Assumes RCS but PP is in FCS!
                _currAlgType = moveWhileTurning;
            }
            break;

            case turnThenMove:
            {
                _algorithm = new cAbstractPathPlanning(this);
                _algorithm->_ppBlocks.push_back(new cXYTrajectory(this));
                _algorithm->_ppBlocks.push_back(new cBoundaryLimiter(this));
                _algorithm->_ppBlocks.push_back(new cXYPID(this));
                _algorithm->_ppBlocks.push_back(new cPhiPID(this));
                _algorithm->_ppBlocks.push_back(new cXYTurnThenMove(this));
                _currAlgType = turnThenMove;
            }
            break;

            case moveThenTurn:
            {
                _algorithm = new cAbstractPathPlanning(this);
                _algorithm->_ppBlocks.push_back(new cXYTrajectory(this));
                _algorithm->_ppBlocks.push_back(new cBoundaryLimiter(this));
                _algorithm->_ppBlocks.push_back(new cXYPID(this));
                _algorithm->_ppBlocks.push_back(new cPhiLinear(this));
                _algorithm->_ppBlocks.push_back(new cPhiMoveThenTurn(this));
                _currAlgType = moveThenTurn;
            }
            break;

            case moveAtSpeed:
            {
                _algorithm = new cAbstractPathPlanning(this);
                _algorithm->_ppBlocks.push_back(new cBoundaryLimiter(this));
                _algorithm->_ppBlocks.push_back(new cMoveAtSpeed(this));
                _currAlgType = moveAtSpeed;
            }
            break;

            case turn:
            {
                _algorithm = new cAbstractPathPlanning(this);
                _algorithm->_ppBlocks.push_back(new cBoundaryLimiter(this));
                _algorithm->_ppBlocks.push_back(new cPhiPID(this));
                _currAlgType = turn;
            }
            break;

            default:
                break;
        }
    }

}

void cPathPlanningMain::limitVelocities(const double &dt, const Position2D &targetPosition, const Position2D &currentPosition, Velocity2D &velocity)
{
    TRACE("> %s", velocity.tostr());
    Velocity2D delta_velocity = velocity - _prev_vel;
    TRACE("previous = %s", _prev_vel.tostr());
    TRACE("new_raw  = %s", velocity.tostr());
    TRACE("delta    = %s", delta_velocity.tostr());
    TRACE("dt       = %12.9f", dt);

    bool has_ball;
    _ppData->getHaveBall(has_ball);

    /* Verify XY tolerance */
    pp_limiters_struct_t limits;
    _ppData->getLimits(limits);
    if((targetPosition.xy() - currentPosition.xy()).size() < limits.tolerationXY)
    {
        velocity.x = 0.0;
        velocity.y = 0.0;
        TRACE("XY limiter: close enough, not moving");
    }
    else
    {
        Vector2D speed_new = velocity.xy();

        /* Cap XY acceleration */
        Vector2D acc_xy = delta_velocity.xy() / dt;

        // Compute dot product between velocity and (de/ac)celeration to determine if it is acceleration or deceleration
        double dotProd = acc_xy * speed_new;

        if (dotProd >= 0.0 && acc_xy.size() > limits.maxAccXY)
        {
            acc_xy.normalize(); // make size equal to 1
            acc_xy = acc_xy * limits.maxAccXY; // make size equal to intended
            speed_new = _prev_vel.xy() + acc_xy * dt;
                TRACE("XY limiter: clipped acceleration setpoint: ax=%6.2f ay=%6.2f vx=%6.2f vy=%6.2f", acc_xy.x, acc_xy.y, speed_new.x, speed_new.y);
        }

        /* Cap XY velocity */
        if (has_ball)
        {
            if (speed_new.size() > limits.maxVelXY_withBall)
            {
                speed_new.normalize(); // make size equal to 1
                speed_new = speed_new * limits.maxVelXY_withBall; // make size equal to intended
                    TRACE("XY limiter: clipped speed WITH BALL setpoint: vx=%6.2f vy=%6.2f", speed_new.x, speed_new.y);
            }
        }
        else
        {
            if (speed_new.size() > limits.maxVelXY)
            {
                speed_new.normalize(); // make size equal to 1
                speed_new = speed_new * limits.maxVelXY; // make size equal to intended
                    TRACE("XY limiter: clipped speed WITHOUT BALL setpoint: vx=%6.2f vy=%6.2f", speed_new.x, speed_new.y);
            }
        }

        velocity.x = speed_new.x;
        velocity.y = speed_new.y;
    }

    /* Verify phi tolerance */
    if(fabs(project_angle_mpi_pi(targetPosition.phi - currentPosition.phi)) < limits.tolerationPhi)
    {
        velocity.phi = 0.0;
        TRACE("angular limiter: close enough, not turning");
    }
    else
    {
        double rotationspeed_new = velocity.phi;

        /* Cap angular acceleration (not deceleration!)*/
        double acc_phi = delta_velocity.phi / dt;
        if (fabs(velocity.phi) >= fabs(_prev_vel.phi) && fabs(acc_phi) > limits.maxAccPhi)
        {
            acc_phi = acc_phi / fabs(acc_phi) * limits.maxAccPhi;
            rotationspeed_new = _prev_vel.phi + acc_phi * dt;
            TRACE("angular limiter: clipped acceleration setpoint: aphi=%6.2f", acc_phi);
        }

        /* Cap angular velocity */
        if (has_ball)
        {
            if (fabs(rotationspeed_new) > limits.maxVelPhi_withBall)
            {
                rotationspeed_new = rotationspeed_new / fabs(rotationspeed_new) * limits.maxVelPhi_withBall;
                TRACE("angular limiter: clipped speed WITH BALL setpoint: vphi=%6.2f", rotationspeed_new);
            }
        }
        else
        {
            if (fabs(rotationspeed_new) > limits.maxVelPhi)
            {
                rotationspeed_new = rotationspeed_new / fabs(rotationspeed_new) * limits.maxVelPhi;
                TRACE("angular limiter: clipped speed WITHOUT BALL setpoint: vphi=%6.2f", rotationspeed_new);
            }
        }

        velocity.phi = rotationspeed_new;
    }
    pp_plot_data_struct_t plotData;
    _ppData->getPlotData(plotData);
    plotData.vel_t.x = velocity.x;
    plotData.vel_t.y = velocity.y;
    plotData.vel_t.phi = velocity.phi;
    _ppData->publishPlotData(plotData);


    /* Save velocity for next iteration acceleration clipping */
    _prev_vel = velocity;

    TRACE("< %s", velocity.tostr());
}

