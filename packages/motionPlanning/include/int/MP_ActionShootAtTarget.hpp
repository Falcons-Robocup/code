// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionShootAtTarget.hpp
 *
 *  Created on: Nov 18, 2017
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONSHOOTATTARGET_HPP_
#define MP_ACTIONSHOOTATTARGET_HPP_

#include "MP_AbstractAction.hpp"


enum class phaseEnum
{
    UNDEFINED = 0,
    ROTATE,
    SETTLE,
    SHOOT,
    FINISHED
};


class MP_ActionShootAtTarget: public MP_AbstractAction
{

public:
    MP_ActionShootAtTarget();
    ~MP_ActionShootAtTarget();
    actionResultTypeEnum execute();

protected:
    // overridden by cActionPassToTarget
    virtual void getSpecificConfig();
    virtual void executeShot();
    virtual void unpackParameters();
    
    void initialize() override;

private:
    // lots of tiny helper functions
    void getCommonConfig();
    void traceData();
    void traceResult(bool success, const char* details);
    void calculateAngles();
    bool checkOk();
    void finalize();
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
    bool       _aimWithinTolerance;
    bool       _settledOk;
    double     _timestamp;
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

#endif /* MP_ACTIONSHOOTATTARGET_HPP_ */

