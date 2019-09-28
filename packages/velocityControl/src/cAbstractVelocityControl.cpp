 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractVelocityControl.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/cAbstractVelocityControl.hpp"
#include <boost/thread/thread.hpp>

cAbstractVelocityControl::cAbstractVelocityControl(cVelocityControlMain* main)
{
    TRACE(">");
    _vcMain = main;
    _dt = 0.0; // Recalculated every iteration due to event-based velocityControl.
    _prevTimestamp = rtime::now();
    _prev_vel = Velocity2D();
    TRACE("<");
}

cAbstractVelocityControl::~cAbstractVelocityControl()
{
}

void cAbstractVelocityControl::execute()
{
    TRACE_FUNCTION("");


    std::list<cAbstractVelocityControl*>::iterator it;
    for (it = _vcBlocks.begin(); it != _vcBlocks.end(); ++it)
    {
        // Compute new _dt for each algorithm
        //(*it)->computeDt();

        //(*it)->setData(_vcData);
        (*it)->execute();
        //_vcData = (*it)->getData();
    }

    //_main->_vcDataClass->publishSpeed(_vcData.vel);
}

void cAbstractVelocityControl::setData(vc_data_struct_t &vcData)
{
    _vcData = vcData;
}

vc_data_struct_t cAbstractVelocityControl::getData()
{
    return _vcData;
}

void cAbstractVelocityControl::computeDt()
{
    // Compute new _dt

    rtime time_now = rtime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside

    _dt = double(time_now - _prevTimestamp);

    // If pathplanning is not used (e.g., robot is stopped), _dt is not recomputed
    // The next time pathplanning is triggered, _dt will be very large (seconds).
    // If this happens, set _dt to 1/30.
    if (_dt > 0.2)
    {
        _dt = 1.0/30.0;
    }

    _prevTimestamp = time_now;
}
