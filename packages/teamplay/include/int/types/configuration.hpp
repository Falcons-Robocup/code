 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuration.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

#include <map>
#include <vector>

#include "configTeamplay.hpp" // sharedTypes
#include "ext/heightmapNames.hpp"
#include "int/types/heightmapEnumTypes.hpp"


namespace teamplay
{

class configuration {
public:
    configuration();
    virtual ~configuration();

    virtual void update(const configTeamplay&);

    virtual float getSetPieceExecuteTimeoutSeconds() const;
    virtual float getPenaltyExecuteTimeoutSeconds() const;
    virtual float getMinKickDistanceKickedMeters() const;
    virtual float getMinPenaltyDistanceKickedMeters() const;
    virtual float getMinOwnKickoffDistanceKickedMeters() const;
    virtual float getShootPathWidth() const;
    virtual float getStraightShotThreshold() const;
    virtual float getAimForCornerThreshold() const;
    virtual float getMinimumAngleToGoal() const;
    virtual float getMaximumAngleToGoal() const;
    virtual float getMinimumDistanceToGoal() const;
    virtual float getMaximumDistanceToGoal() const;
    virtual float getInterceptBallCaptureRadius() const;
    virtual float getInterceptBallMinimumSpeed() const;
    virtual bool isActiveInterceptEnabled() const;
    virtual bool getDefendingStrategy() const;
    virtual float getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const heightmapEnum& heightmap) const;

private:
    configTeamplay _config;
};


} /* namespace teamplay */

#endif /* CONFIGURATION_HPP_ */
