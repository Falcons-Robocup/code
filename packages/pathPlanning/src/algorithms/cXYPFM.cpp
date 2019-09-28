 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPIDPathPlanning.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: Jan Feitsma
 */

#include "int/algorithms/cXYPFM.hpp"

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYPFM::execute()
{
    TRACE(">");
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get obstacles from cPathPlanningData
    std::vector<pp_obstacle_struct_t> obstacles;
    _main->_ppData->getAllObstacles(obstacles);

    // Get PFM parameters (reconfig) from cPathPlanningData
    pp_pfm_params_struct_t pfmParams;
    _main->_ppData->getPFMParams(pfmParams);

    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);


    TRACE("PFM update target%s", _ppData.pos.tostr());
    TRACE("limits.maxAccXY     = %6.2f", limits.maxAccXY);

    Velocity2D newVel;

    /* Calculate PFM attractive force, for now simply linear */
    // TODO: combine with PIDPathPlanning
    Vector2D f_attractive = _ppData.pos.xy() - currPos.xy();
    // clipping
    double ATTRACTIVE_MULTIPLIER = 2.0;
    double f_attr_max = pfmParams.force_max * ATTRACTIVE_MULTIPLIER;
    if (f_attractive.size() > f_attr_max)
    {
        f_attractive.normalize(f_attr_max);
    }
    if (f_attractive.size() < pfmParams.force_min)
    {
        f_attractive *= 0;
    }
    TRACE("PFM attractiveForce x=%6.2f y=%6.2f", f_attractive.x, f_attractive.y);

    /* Calculate repulsive force */
    Vector2D f_repulsive;
    for(int i = 0; i<(int)obstacles.size(); ++i)
    {
                Vector2D obst_pos(obstacles[i].location.x, obstacles[i].location.y);
        TRACE("PFM checking obstacle %d: x=%6.2f y=%6.2f", i, obst_pos.x, obst_pos.y);
        // ignore current and obstacle speed for now
        Vector2D delta_pos = currPos.xy() - obst_pos;
        double denom = pow(delta_pos.size(), pfmParams.expo_rep);
                if (denom < 0.0001) denom = 0.0001; // prevent div by zero

        // distance cutoff: ignore any obstacle which is too far away
        if (delta_pos.size() >= pfmParams.dist_lim) continue;

        // calculate repulsive force
        Vector2D f_repulsive_this = pfmParams.mult_rep * delta_pos / denom;

        // scale repulsive force with delta_speed
        // to better be able to reach target
        Vector2D delta_speed = _prev_vel.xy() - Vector2D(obstacles[i].velocity.x, obstacles[i].velocity.y);
        Vector2D target_obst_delta_pos = obst_pos - _ppData.pos.xy();
        double repulsive_scaling = pow(delta_speed.size(), 2);
        if (target_obst_delta_pos.size() < 1)
        {
            repulsive_scaling *= pow(target_obst_delta_pos.size(), 2);
        }
        f_repulsive_this *= repulsive_scaling;

                // clip force per obstacle
        if (f_repulsive_this.size() > pfmParams.force_max)
        {
            f_repulsive_this.normalize(pfmParams.force_max);
        }

        // check if robot has passed the obstacle, in which case it should not
        // be pushed away anymore. Use inner product, which must be positive (=driving towards)
        double innerprod = f_repulsive_this * delta_speed;
        TRACE("delta speed: %6.2f y=%6.2f   repulsive_scaling=%6.2f  innerprod=%6.2f", delta_speed.x, delta_speed.y, repulsive_scaling, innerprod);    TRACE("delta speed: %6.2f y=%6.2f   repulsive_scaling=%6.2f  innerprod=%6.2f", delta_speed.x, delta_speed.y, repulsive_scaling, innerprod);
        //if (innerprod < 0) this is shaky on robot....
        if (1)
        {
            // truncate too small force
            if (f_repulsive_this.size() > pfmParams.force_min)
            {
                TRACE("PFM avoiding obstacle %d: x=%6.2f y=%6.2f", i, f_repulsive_this.x, f_repulsive_this.y);
                    f_repulsive += f_repulsive_this;

                // Add tracing for visualizer to display the forces per obstacle
                TRACE("VisualizeObstacleForce %6.2f %6.2f %6.2f %6.2f", obst_pos.x, obst_pos.y, f_repulsive_this.x * pfmParams.gain_rep, f_repulsive_this.y * pfmParams.gain_rep);
            } // otherwise ignore this small contribution.
        }
    }
    TRACE("PFM repulsiveForce x=%6.2f y=%6.2f", f_repulsive.x, f_repulsive.y);
    TRACE("PFM attractiveForce x=%6.2f y=%6.2f", f_attractive.x, f_attractive.y);

    // translate netto force to XY accelleration and speed
    Vector2D f_total = pfmParams.gain_attr * f_attractive + pfmParams.gain_rep * f_repulsive;
    TRACE("VisualizeRobotForce %6.2f %6.2f %6.2f %6.2f", currPos.x, currPos.y, pfmParams.gain_attr * f_attractive.x, pfmParams.gain_attr * f_attractive.y);

    // avoidance trick
    double intended_size = pfmParams.gain_attr * f_attractive.size();
        if ((f_total.size() < intended_size) && (intended_size > 0.1))
    {
        f_total.normalize(intended_size);
    }

    TRACE("PFM f_total x=%6.2f y=%6.2f", f_total.x, f_total.y);

    Vector2D acc_tot = f_total / 36.0; // divide by mass
    TRACE("acc: ax=%6.2f ay=%6.2f", acc_tot.x, acc_tot.y);
    // note: acceleration and speed clipping is done in limitVelocities

    // achieve braking behavior by treating acceleration as speed
    //Vector2D new_vel_xy = _prev_vel.xy() + acc_tot * _dt;
    Vector2D new_vel_xy = acc_tot;

    // angular pathplanning for now simply linear
    //newVel.phi = project_angle_mpi_pi(_main->_tpAdapter._targetPosition.phi - _main->_currPos.phi);

    /* Limit the velocity and publish */
    newVel.x = new_vel_xy.x;
    newVel.y = new_vel_xy.y;

    _ppData.vel.x = newVel.x;
    _ppData.vel.y = newVel.y;
    TRACE("<");
}
