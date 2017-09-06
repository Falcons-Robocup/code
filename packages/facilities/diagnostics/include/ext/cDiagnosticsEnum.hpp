 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsEnum.hpp
 *
 *  Created on: Jan 09, 2016
 *      Author: Jan Feitsma
 *
 * TODO: generate python wrapper?
 */

#ifndef DIAGNOSTICS_ENUM_HPP_
#define DIAGNOSTICS_ENUM_HPP_


namespace diagnostics
{

// HOW TO DEFINE A NEW DIAGNOSTICS CHANNEL
// * add your new enum value here, before DIAG_LAST
// * register the associated topic and ROS msg type in cDiagnosticsReceiver constructor
// * done! now, for each robot a listener on coach will be instantiated and a topic will appear; 
//   probably you also want to do something in analyzer / visualizer ... TBD

enum diagEnum
{ 
    DIAG_INVALID = 0, 
    DIAG_CONTROL = 1,           // re-defined in python (diagnostics/test/stress_test.py), don't change numeric value!
    DIAG_VISION = 2,            // re-defined in python (diagnostics/test/stress_test.py), don't change numeric value!
    DIAG_HEALTH_FAST,
    DIAG_HEALTH_MID,
    DIAG_HEALTH_SLOW,
    DIAG_PATHPLANNING, 
    DIAG_TEAMPLAY, 
    DIAG_WORLDMODEL,            // note that there is also a topic g_worldmodel on coach level 
    DIAG_ERROR,
    DIAG_INFO, 
    DIAG_HALMW,
    DIAG_COMPASS,
    DIAG_ACTIVE,                // obsolete
    DIAG_FRONTVISION,
    DIAG_REFBOX,
    DIAG_VISION_RFRAME,
    DIAG_WM_LOC,
    DIAG_WM_BALL,
    DIAG_WM_OBST,
    DIAG_WM_TOP,
    DIAG_EVENTS,
    DIAG_LAST // last entry, to be able to determine the number of diagnostics listeners to instantiate
};

} // end of namespace diagnostics

#endif /* DIAGNOSTICS_ENUM_HPP_ */
