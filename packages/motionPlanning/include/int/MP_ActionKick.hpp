// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionKick.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONKICK_HPP_
#define MP_ACTIONKICK_HPP_

#include "MP_ActionShootAtTarget.hpp"


// mainly intended for calibration and possibly self-pass
// to mimic the ballHandler-release timing, so calibration uses consistent timing
// TODO: factor out shooting timing, now we have some code duplication ...

class MP_ActionKick: public MP_AbstractAction
{
public:
    MP_ActionKick();
    actionResultTypeEnum execute();
    void unpackParameters();
private:
    float _kickPower = 0.0;
    float _kickHeight = 0.0;
    bool _actuatedKickerHeight = false;
    bool _actuatedBhDisable = false;
    cTimer _heightTimer;
    cTimer _bhTimer;

};

#endif

