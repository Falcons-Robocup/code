 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cMotionPlanningInterface.hpp
 *
 *  Created on: Nov 25, 2017
 *      Author: Jan Feitsma
 */

#ifndef CMOTIONPLANNINGINTERFACE_HPP_
#define CMOTIONPLANNINGINTERFACE_HPP_

#include "polygon2D.hpp"
#include "FalconsCommon.h"

enum class mpStatusEnum
{
    UNDEFINED = 0,
    RUNNING,
    PASSED,
    FAILED,
    ERROR
};

class cMotionPlanningInterface
{
public:
    cMotionPlanningInterface() {};
    virtual ~cMotionPlanningInterface() {};

    virtual void connect () = 0;

    virtual mpStatusEnum moveTo(Position2D targetPos, const std::vector<polygon2D>& forbiddenAreas = {}, bool slow = false) = 0;
    virtual mpStatusEnum passTo(float x, float y) = 0;
    virtual mpStatusEnum shootAt(float x, float y, float z) = 0;
    virtual mpStatusEnum lobShot(float x, float y, float z) = 0;
    virtual void stop() = 0;
    virtual void suppressBallHandlers() = 0;
    virtual mpStatusEnum getBall(bool slow) = 0;

};

#endif /* CMOTIONPLANNINGINTERFACE_HPP_ */
