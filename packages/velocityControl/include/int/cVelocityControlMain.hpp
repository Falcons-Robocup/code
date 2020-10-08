 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#ifndef CVELOCITYCONTROLMAIN_HPP_
#define CVELOCITYCONTROLMAIN_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "falconsCommon.hpp"

#include "int/cVelocityControlTypes.hpp"
#include "int/cVelocityControlData.hpp"
//#include "int/cWorldModelAdapter.hpp"
//#include "int/cTeamplayAdapter.hpp"
//#include "int/cSetSpeedAdapter.hpp"
//#include "int/cReconfigureAdapter.hpp"
//#include "int/cDiagnosticsAdapter.hpp"

// Forward declarations > include in include
class cAbstractVelocityControl;

/*!
 * \brief The main VelocityControl class
 */
class cVelocityControlMain
{
    public:
        /*!
        * \brief The constructor of cVelocityControlMain
        */
        cVelocityControlMain();
        //        cVelocityControlMain(cVelocityControlData& data);

        /*!
        * \brief The destructor of cVelocityControlMain
        */
        ~cVelocityControlMain();

        /*!
        * \brief Performs a single iteration of the VelocityControl work.
        */
        void iterateFeedback();
        void iterateSetpoint();

        void setAlgorithms();

        cVelocityControlData* _vcDataClass;


    private:

        /*! \brief The algorithm sequence used */
        cAbstractVelocityControl *_feedbackAlgorithm;
        cAbstractVelocityControl *_setpointAlgorithm;

        /*! \brief The current active algorithm type */
        vc_algorithm_type _currAlgType;


};


#endif /* CVELOCITYCONTROLMAIN_HPP_ */
