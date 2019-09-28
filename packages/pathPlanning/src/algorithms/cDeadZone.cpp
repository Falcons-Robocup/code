 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDeadZone.cpp
 *
 *  Created on: 2017-11-21
 *      Author: Erik Kouters
 */

#include "int/algorithms/cDeadZone.hpp"

void cDeadZone::execute()
{
    TRACE(">");

    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    /* Verify XY tolerance */
    if (fabs(_ppData.pos.x - currPos.x) < limits.tolerationXY)
    {
        _ppData.vel.x = 0.0;
        TRACE("X limiter: close enough, not moving in X: currPos=%12.9f, targetPos=%12.9f", currPos.x, _ppData.pos.x);
    }
    if (fabs(_ppData.pos.y - currPos.y) < limits.tolerationXY)
    {
        _ppData.vel.y = 0.0;
        TRACE("Y limiter: close enough, not moving in Y: currPos=%12.9f, targetPos=%12.9f", currPos.y, _ppData.pos.y);
    }

    /* Verify phi tolerance */
    if(fabs(project_angle_mpi_pi(_ppData.pos.phi - currPos.phi)) < limits.tolerationPhi)
    {
        _ppData.vel.phi = 0.0;
        TRACE("angular limiter: close enough, not turning: currPos=%12.9f, targetPos=%12.9f", currPos.phi, _ppData.pos.phi);
    }

    TRACE("<");
}
