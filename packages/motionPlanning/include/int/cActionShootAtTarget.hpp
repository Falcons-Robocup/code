 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionShootAtTarget.hpp
 *
 *  Created on: Nov 18, 2017
 *      Author: Jan Feitsma
 */

#ifndef CACTIONSHOOTATTARGET_HPP_
#define CACTIONSHOOTATTARGET_HPP_

#include "int/cAbstractAction.hpp"


enum class phaseEnum
{
    UNDEFINED = 0,
    ROTATE,
    SETTLE,
    SHOOT,
    FINISHED
};


class cActionShootAtTarget: public cAbstractAction
{

public:
    cActionShootAtTarget();
    ~cActionShootAtTarget();
    actionResultTypeEnum execute();

protected:
    // overridden by cActionPassToTarget
    virtual void getSpecificConfig();
    virtual void executeShot();
    virtual void unpackParameters();
    
    void initialize();

private:
    // lots of tiny helper functions
    void getCommonConfig();
    void traceData();
    void traceResult(bool success, const char* details);
    void calculateAngles();
    bool checkOk();
    void finalize();
    void checkDisableBh();
    void shoot();
    void moveToTarget();
    void enterPhaseRotate();
    void enterPhaseSettle();
    void enterPhaseShoot();
    void enterPhaseFinished();
    void determineCurrentPhase();
    void executeCurrentPhase();
    std::string phaseString();
    
    // data members
    actionResultTypeEnum _result;
    bool       _didShoot;
    bool       _disabledBh;
    bool       _aimWithinTolerance;
    bool       _settledOk;
    double     _timestamp;
    cTimer     _bhTimer;
    cTimer     _shootTimer;
    cTimer     _aimTimer;
    cTimer     _settleTimer;
    Position2D _currentPos;
    Position2D _targetPos;
    float      _deltaPhi;
    float      _initialDeltaPhi;
    float      _moveAngleThreshold;
    float      _aimSettleTime;
    float      _coarseAngle;
    float      _disableBhDelay;
    float      _sleepAfterShoot;
    phaseEnum  _currentPhase;
    phaseEnum  _previousPhase;

protected:
    // overridable / accessible (for passToTarget)
    Point3D    _shootTargetPos;
    actionTypeEnum _shootType;
    float      _distance;
    float      _accuracy;
    float      _timeout;
    std::string _actionName;
    
};

#endif /* CACTIONSHOOTATTARGET_HPP_ */

