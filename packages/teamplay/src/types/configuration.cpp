 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include "int/utilities/trace.hpp"

using namespace teamplay;

configuration::configuration()
: _heightMapsGeneratePictures(false)
{
}

configuration::~configuration()
{
}

float configuration::getSetPieceExecuteTimeoutSeconds() const
{
    return _setpieceExecuteTimeoutSeconds;
}

float configuration::getPenaltyExecuteTimeoutSeconds() const
{
    return _penaltyExecuteTimeoutSeconds;
}

float configuration::getMinKickDistanceKickedMeters() const
{
    return _minKickDistanceKickedMeters;
}

float configuration::getMinPenaltyDistanceKickedMeters() const
{
    return _minPenaltyDistanceKickedMeters;
}

float configuration::getMinOwnKickoffDistanceKickedMeters() const
{
    return _minOwnKickoffDistanceKickedMeters;
}

bool configuration::isRuleStimulatePassingEnabled() const
{
    return _ruleStimulatePassingEnabled;
}

float configuration::getShootPathWidth() const
{
    return _shootPathWidth;
}

float configuration::getStraightShotThreshold() const
{
    return _straightShotThreshold;
}

float configuration::getAimForCornerThreshold() const
{
    return _aimForCornerThreshold;
}

float configuration::getShotThreshold() const
{
    return _shotThreshold;
}

float configuration::getInterceptBallCaptureRadius() const // TODO cleanup? (moved to motionPlanning?)
{
    return _interceptBallCaptureRadius;
}

float configuration::getInterceptBallMinimumSpeed() // TODO cleanup? (moved to motionPlanning?)
{
    return _interceptBallMinimumSpeed;
}

bool configuration::getDefendingStrategy()
{
    return _defendingStrategy;
}

bool configuration::getDribbleStrategy()
{
    return _dribbleStrategy;
}

bool configuration::getHeightMapsGeneratePictures()
{
    return _heightMapsGeneratePictures;
}

float configuration::getHeightMapFactorForAction (const tpActionEnum& action, const heightmapEnum& heightmap) const
{
    auto factors = _heightMapContributionFactors.find(action);
    if (factors != _heightMapContributionFactors.end())
    {
        auto factor = factors->second.find(heightmap);
        if (factor != factors->second.end())
        {
            return factor->second;
        }
    }

    TRACE("Factor not found. Returning zero.");
    return 0.0;
}

void configuration::setSetPieceExecuteTimeout(const int t)
{
    _setpieceExecuteTimeoutSeconds = t;
}

void configuration::setPenaltyExecuteTimeout(const int t)
{
    _penaltyExecuteTimeoutSeconds = t;
}

void configuration::setMinKickDistanceKickedMeters(const float m)
{
    _minKickDistanceKickedMeters = m;
}

void configuration::setMinPenaltyDistanceKickedMeters(const float m)
{
    _minPenaltyDistanceKickedMeters = m;
}

void configuration::setMinOwnKickoffDistanceKickedMeters(const float m)
{
    _minOwnKickoffDistanceKickedMeters = m;
}

void configuration::setRuleStimulatePassing(const bool enabled)
{
    _ruleStimulatePassingEnabled = enabled;
}

void configuration::setShootPathWidth (const float widthInMeters)
{
    _shootPathWidth = widthInMeters;
}

void configuration::setStraightShotThreshold (const float distanceInMeters)
{
    _straightShotThreshold = distanceInMeters;
}

void configuration::setAimForCornerThreshold (const float distanceInMeters)
{
    _aimForCornerThreshold = distanceInMeters;
}

void configuration::setShotThreshold (const float distanceInMeters)
{
    _shotThreshold = distanceInMeters;
}

void configuration::setInterceptBallCaptureRadius (const float captureRadiusInMeters)
{
    _interceptBallCaptureRadius = captureRadiusInMeters;
}

void configuration::setInterceptBallMinimumSpeed (const float minimumSpeedInMetersPerSecond)
{
    _interceptBallMinimumSpeed = minimumSpeedInMetersPerSecond;
}

void configuration::setDefendingStrategy(const bool defendingStrategy)
{
    _defendingStrategy = defendingStrategy;
}

void configuration::setDribbleStrategy(const bool dribbleStrategy)
{
    _dribbleStrategy = dribbleStrategy;
}

void configuration::setHeightMapsGeneratePictures(const bool heightMapsGeneratePictures)
{
    _heightMapsGeneratePictures = heightMapsGeneratePictures;
}

void configuration::setHeightMapFactorForAction (const tpActionEnum& action, const heightmapEnum& heightmap, const float factor)
{
    _heightMapContributionFactors[action][heightmap] = factor;
}
