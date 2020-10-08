 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuration.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#include "int/types/configuration.hpp"
#include "ext/heightmapNames.hpp"
#include "tracing.hpp"

using namespace teamplay;

configuration::configuration()
{
}

configuration::~configuration()
{
}

void configuration::update (const configTeamplay& c)
{
    _config = c;
}

float configuration::getSetPieceExecuteTimeoutSeconds() const
{
    return _config.rules.setpieceExecuteTimeout;
}

float configuration::getPenaltyExecuteTimeoutSeconds() const
{
    return _config.rules.penaltyExecuteTimeout;
}

float configuration::getMinKickDistanceKickedMeters() const
{
    return _config.rules.minKickDistanceKicked;
}

float configuration::getMinPenaltyDistanceKickedMeters() const
{
    return _config.rules.minPenaltyDistanceKicked;
}

float configuration::getMinOwnKickoffDistanceKickedMeters() const
{
    return _config.rules.minOwnKickoffDistanceKicked;
}

float configuration::getShootPathWidth() const
{
    return _config.shooting.shootPathWidth;
}

float configuration::getStraightShotThreshold() const
{
    return _config.shooting.straightShotThreshold;
}

float configuration::getAimForCornerThreshold() const
{
    return _config.shooting.aimForCornerThreshold;
}

float configuration::getMinimumAngleToGoal() const
{
    return _config.shooting.angleToGoal.minimum;
}

float configuration::getMaximumAngleToGoal() const
{
    return _config.shooting.angleToGoal.maximum;
}

float configuration::getMinimumDistanceToGoal() const
{
    return _config.shooting.distanceToGoal.minimum;
}

float configuration::getMaximumDistanceToGoal() const
{
    return _config.shooting.distanceToGoal.maximum;
}

float configuration::getInterceptBallCaptureRadius() const // TODO cleanup? (moved to motionPlanning?)
{
    return _config.interceptBall.captureRadius;
}

float configuration::getInterceptBallMinimumSpeed() const // TODO cleanup? (moved to motionPlanning?)
{
    return _config.interceptBall.minimumSpeed;
}

bool configuration::isActiveInterceptEnabled() const
{
    return _config.interceptBall.activeIntercept;
}

bool configuration::getDefendingStrategy() const
{
    return _config.strategy.defendingStrategy;
}

float configuration::getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const heightmapEnum& heightmap) const
{
    auto factors = _config.heightmaps.factors.find(enum2str(compositeHeightmap));
    if (factors != _config.heightmaps.factors.end())
    {
        auto factor = factors->second.find(enum2str(heightmap));
        if (factor != factors->second.end())
        {
            return factor->second;
        }
    }

    TRACE("Factor not found. Returning zero.");
    return 0.0;
}

