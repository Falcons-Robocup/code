// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMoveToTarget.hpp
 *
 *  Created on: Nov 26, 2017
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONMOVETARGET_HPP_
#define MP_ACTIONMOVETARGET_HPP_

#include "MP_AbstractAction.hpp"


class MP_ActionMoveToTarget: public MP_AbstractAction
{

public:
    MP_ActionMoveToTarget();
    actionResultTypeEnum execute();
    
private:
    void unpackParameters();
    
    // data members
    actionResultTypeEnum _result;
    bool                 _ballHandlersEnabled;
    motionTypeEnum       _motionType = motionTypeEnum::INVALID;
    Position2D           _targetPos;
    
};

#endif /* MP_ACTIONMOVETARGET_HPP_ */

