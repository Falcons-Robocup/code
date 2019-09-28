 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include "tpActionEnum.hpp" // sharedTypes
#include "int/types/heightmapEnumTypes.hpp"


namespace teamplay
{

class configuration {
public:
    configuration();
    virtual ~configuration();

    virtual void setSetPieceExecuteTimeout(const int timeoutMilliseconds);
    virtual float getSetPieceExecuteTimeoutSeconds() const;

    virtual void setPenaltyExecuteTimeout(const int timeoutMilliseconds);
    virtual float getPenaltyExecuteTimeoutSeconds() const;

    virtual void setMinKickDistanceKickedMeters(const float distanceMeters);
    virtual float getMinKickDistanceKickedMeters() const;

    virtual void setMinPenaltyDistanceKickedMeters(const float distanceMeters);
    virtual float getMinPenaltyDistanceKickedMeters() const;

    virtual void setMinOwnKickoffDistanceKickedMeters(const float distanceMeters);
    virtual float getMinOwnKickoffDistanceKickedMeters() const;

    virtual void setRuleStimulatePassing(const bool enabled);
    virtual bool isRuleStimulatePassingEnabled() const;

    virtual void setShootPathWidth (const float widthInMeters);
    virtual float getShootPathWidth() const;

    virtual void setStraightShotThreshold (const float distanceInMeters);
    virtual float getStraightShotThreshold() const;

    virtual void setAimForCornerThreshold (const float distanceInMeters);
    virtual float getAimForCornerThreshold() const;

    virtual void setShotThreshold (const float distanceInMeters);
    virtual float getShotThreshold() const;

    virtual void setInterceptBallCaptureRadius (const float captureRadiusInMeters);
    virtual float getInterceptBallCaptureRadius() const;

    virtual void setInterceptBallMinimumSpeed (const float minimumSpeedInMetersPerSecond);
    virtual float getInterceptBallMinimumSpeed();

    virtual void setDefendingStrategy (const bool defendingStrategy);
    virtual bool getDefendingStrategy();

    virtual void setDribbleStrategy (const bool dribbleStrategy);
    virtual bool getDribbleStrategy();

    virtual void setHeightMapsGeneratePictures (const bool heightMapsGeneratePictures);
    virtual bool getHeightMapsGeneratePictures();

    virtual void setHeightMapFactorForAction (const tpActionEnum& action, const heightmapEnum& heightmap, const float factor);
    virtual float getHeightMapFactorForAction (const tpActionEnum& action, const heightmapEnum& heightmap) const;

private:
    float _setpieceExecuteTimeoutSeconds;
    float _penaltyExecuteTimeoutSeconds;
    float _minKickDistanceKickedMeters;
    float _minPenaltyDistanceKickedMeters;
    float _minOwnKickoffDistanceKickedMeters;
    bool  _ruleStimulatePassingEnabled;
    float _shootPathWidth;
    float _straightShotThreshold;
    float _aimForCornerThreshold;
    float _shotThreshold;
    float _interceptBallCaptureRadius;
    float _interceptBallMinimumSpeed;
    bool _defendingStrategy;
    bool _dribbleStrategy;
    bool _heightMapsGeneratePictures;
    std::map<tpActionEnum, std::map<heightmapEnum, float> > _heightMapContributionFactors;
};


} /* namespace teamplay */

#endif /* CONFIGURATION_HPP_ */
