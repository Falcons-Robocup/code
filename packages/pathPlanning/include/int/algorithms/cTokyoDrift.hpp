 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTokyoDrift.hpp
 *
 *  Created on: 2017-03-02
 *      Author: Erik Kouters
 */

#ifndef CTOKYODRIFT_HPP_
#define CTOKYODRIFT_HPP_

#include <string>

#include "int/cAbstractPathPlanning.hpp"

class cTokyoDrift : public cAbstractPathPlanning
{
    public:
        /*!
         * \brief The constructor of cTokyoDrift
         *
         * \param[in] main The parent class
         */
        cTokyoDrift(cPathPlanningMain* main) : cAbstractPathPlanning(main) { };

        /*!
         * \brief The destructor of cTrajectoryPathPlanning
         */
        ~cTokyoDrift() {};

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

#endif /* CTOKYODRIFT_HPP_ */
