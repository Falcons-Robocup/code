// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionTurnAwayFromOpponent.hpp
 *
 *  Created on: Apr 26, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_ACTIONTURNAWAYFROMOPPONENT_HPP_
#define MP_ACTIONTURNAWAYFROMOPPONENT_HPP_

#include "MP_AbstractAction.hpp"


class MP_ActionTurnAwayFromOpponent: public MP_AbstractAction
{

public:
    MP_ActionTurnAwayFromOpponent();
    actionResultTypeEnum execute();

    void initialize() override;
    
private:
    void unpackParameters();
    void calculate();
    
    // data members
    Position2D _currentPos;
    Position2D _opponentPos;
    Position2D _targetPos;
    
};

#endif /* MP_ACTIONTURNAWAYFROMOPPONENT_HPP_ */

