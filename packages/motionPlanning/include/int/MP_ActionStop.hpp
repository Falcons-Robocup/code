// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionStop.hpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONSTOP_HPP_
#define MP_ACTIONSTOP_HPP_

#include "MP_AbstractAction.hpp"

class MP_ActionStop: public MP_AbstractAction
{
public:
    actionResultTypeEnum execute();
    void unpackParameters();
private:
    bool _ballHandlersEnabled;
};

#endif /* MP_ACTIONSTOP_HPP_ */

