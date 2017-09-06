 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractPathPlanning.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#ifndef CABSTRACTPATHPLANNING_HPP_
#define CABSTRACTPATHPLANNING_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "int/cPathPlanningMain.hpp"

/*!
 * \brief is the abstract class that each PathPlanning algorithm should inherit.
 *
 * cAbstractPathPlanning offers the virtual functions updatePathPlanningXY() and updatePathPlanningPhi()
 * that decouple XY movement from turning movement. This allows the PathPlanning algorithm
 * to specify a different algorithm for XY movement than for turning movement.
 */
class cAbstractPathPlanning
{
    public:
        /*!
         * \brief The constructor of cAbstractPathPlanning
         *
         * \param[in] main The parent class
         */
        cAbstractPathPlanning(cPathPlanningMain* main);

        /*!
         * \brief The destructor of cAbstractPathPlanning
         */
        virtual ~cAbstractPathPlanning();

        virtual void execute();

        void setData(pp_data_struct_t &ppData);
        pp_data_struct_t getData();

        void computeDt();


        /*!
         * \brief Called for every time-interval and publishes the new XY velocities.
         *
         * This function needs to be reimplemented by every PathPlanning algorithm.
         * It is called every time-interval (MOTION_FREQUENCY Hz) and publishes the
         * newly calculated XY velocities to the cSetSpeedAdapter.
         *
         * \retval Velocity2D           The new target velocity of the robot (XY only)
         */
        //virtual Velocity2D updatePathPlanningXY();

        /*!
         * \brief Called for every time-interval and publishes the new turning velocities.
         *
         * This function needs to be reimplemented by every PathPlanning algorithm.
         * It is called every time-interval (MOTION_FREQUENCY Hz) and publishes the
         * newly calculated turning velocities to the cSetSpeedAdapter.
         *
         * \retval Velocity2D           The new target velocity of the robot (turning only)
         */
        //virtual Velocity2D updatePathPlanningPhi();

        std::list<cAbstractPathPlanning*> _ppBlocks;

        /*! \brief The delta time for every interval */
        double             _dt;

    protected:

        /*! \brief The parent class */
        cPathPlanningMain* _main;

        timeval _prevTimestamp;

        /*! \brief The velocity of the previous interval. Often used by the different algorithms. */
        Velocity2D         _prev_vel;

        pp_data_struct_t _ppData;

};


#endif /* CABSTRACTPATHPLANNING_HPP_ */
