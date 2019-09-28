 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTrajectoryPathPlanning.hpp
 *
 *  Created on: Apr 16 2015
 *      Author: Thomas Kerkhof
 *      Largely stolen from: Jan Feitsma
 */

#ifndef CXYTRAJECTORY_HPP_
#define CXYTRAJECTORY_HPP_

#include <string>

#include "int/cAbstractPathPlanning.hpp"

//#define ROBOT_RADIUS 26.0 / 100.0 // 26cm = 0.26m
//#define ROBOT_RADIUS 75.0 / 100.0 / 2.0 // 75cm diameter = 37.5cm radius = 0.375m

/*!
 * \brief TrajectoryPlanning evades obstacles by overriding the target to a subtarget next to the obstacles and then calling PID.
 *
 * Introduction
 * ------------
 * The TrajectoryPlanning algorithm is based on TU/e's paper "Realtime motion path generation using subtargets in a changing environment" by Bruijnen et al.
 * In short, it will look for obstacles in the direct path from robot location to target location.
 * If there are obstacles blocking the robot's path, the algorithm will place a subtarget next to the obstacle(s).
 * This subtarget will then be set as the new target that is sent to PID.
 *
 * Technical Explanation
 * --------------------
 *
 * \image html trajectory_planning.png "Subtarget from Trajectory Planning"
 *
 * Let \f$O\f$ be the list of all obstacles.
 *
 * Get the list of obstacles \f$B\f$ that \f$\forall b \in B\f$ is in the path between robot and target.
 * Let \f$f \in B\f$ be the obstacle which is the closest to the robot.
 *
 * Create the list \f$G\f$:
 * 1. Let \f$G = \lbrace f \rbrace\f$
 * 2. \f$\forall o \in O \land \forall g \in G\f$ if \f$\mbox{dist}(o, g) < \mbox{size}(\mbox{robot})\f$ then add \f$o\f$ to \f$G\f$.
 * 3. Redo the previous step until \f$G\f$ remains the same size.
 *
 * Finally, place the subtarget next to \f$G\f$ on the side that is closest to the target.
 *
 */
class cXYTrajectory : public cAbstractPathPlanning
{
    public:
        /*!
         * \brief The constructor of cTrajectoryPathPlanning
         *
         * \param[in] main The parent class
         */
        cXYTrajectory(cPathPlanningMain* main) : cAbstractPathPlanning(main) { };

        /*!
         * \brief The destructor of cTrajectoryPathPlanning
         */
        ~cXYTrajectory() {};

        void execute();

        /*!
         * \brief Called for every time-interval and publishes the new XY velocities.
         *
         * This function determines the new subtarget and publishes the
         * subtarget to the cPIDPathPlanning algorithm.
         *
         * \retval Velocity2D           The new target velocity of the robot (XY only)
         */
        //virtual Velocity2D updatePathPlanningXY();

        /*!
         * \brief Called for every time-interval and publishes the new turning velocities.
         *
         * This function simply calls the cPIDPathPlanning algorithm.
         *
         * \retval Velocity2D           The new target velocity of the robot (turning only)
         */
        //virtual Velocity2D updatePathPlanningPhi();

};

#endif /* CXYTRAJECTORY_HPP_ */
