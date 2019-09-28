 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractVelocityControl.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CABSTRACTVELOCITYCONTROL_HPP_
#define CABSTRACTVELOCITYCONTROL_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "tracing.hpp"
#include "FalconsRtDB2.hpp" // for rtime

#include "int/cVelocityControlMain.hpp"

/*!
 * \brief is the abstract class that each VelocityControl algorithm should inherit.
 *
 * cAbstractVelocityControl offers the virtual functions updateVelocityControlXY() and updateVelocityControlPhi()
 * that decouple XY movement from turning movement. This allows the VelocityControl algorithm
 * to specify a different algorithm for XY movement than for turning movement.
 */
class cAbstractVelocityControl
{
    public:
        /*!
         * \brief The constructor of cAbstractVelocityControl
         *
         * \param[in] main The parent class
         */
        cAbstractVelocityControl(cVelocityControlMain* main);

        /*!
         * \brief The destructor of cAbstractVelocityControl
         */
        virtual ~cAbstractVelocityControl();

        virtual void execute();

        void setData(vc_data_struct_t &vcData);
        vc_data_struct_t getData();

        void computeDt();

        std::list<cAbstractVelocityControl*> _vcBlocks;

        /*! \brief The delta time for every interval */
        double             _dt;

    protected:

        /*! \brief The parent class */
        cVelocityControlMain* _vcMain;

        rtime _prevTimestamp;

        /*! \brief The velocity of the previous interval. Often used by the different algorithms. */
        Velocity2D         _prev_vel;

        vc_data_struct_t _vcData;

};


#endif /* CABSTRACTVELOCITYCONTROL_HPP_ */
