 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionPassToTarget.cpp
 *
 *  Created on: Nov 26, 2017
 *      Author: Jan Feitsma
 
The action passToTarget is derived from shootAtTarget, to make use of the same phases and timings (e.g. ballHandler release before shoot).
Only the configurables are different, as well as the shoot power (determined by shootPlanning).
 */

#include "../include/int/MP_ActionPassToTarget.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"

MP_ActionPassToTarget::MP_ActionPassToTarget()
{
    TRACE("init start");
    initialize();
    TRACE("init end");
}

void MP_ActionPassToTarget::getSpecificConfig()
{
    _accuracy = getConfig().passToTargetConfig.accuracy;
    _timeout = getConfig().passToTargetConfig.timeout;
    _actionName = "PASS";
}

void MP_ActionPassToTarget::unpackParameters()
{
    _shootTargetPos.x = boost::lexical_cast<float>(_params.at(0));
    _shootTargetPos.y = boost::lexical_cast<float>(_params.at(1));
    _shootTargetPos.z = 0.0;
    _shootType = actionTypeEnum::PASS;
}

void MP_ActionPassToTarget::executeShot()
{
    TRACE_FUNCTION("");
    TRACE("Executing pass");
    _rtdbOutput->setShootSetpoint(shootPhaseEnum::SHOOT, shootTypeEnum::PASS, Position2D(_distance, 0, 0));
}

