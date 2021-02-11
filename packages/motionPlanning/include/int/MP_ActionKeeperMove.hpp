// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionKeeperMove.hpp
 *
 *  Created on: Apr 26, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_ACTIONKEEPERMOVE_HPP_
#define MP_ACTIONKEEPERMOVE_HPP_

#include "MP_AbstractAction.hpp"


class MP_ActionKeeperMove: public MP_AbstractAction
{

public:
    MP_ActionKeeperMove();
    actionResultTypeEnum execute();
    
private:
    void unpackParameters();
    void getCfg();
    void calculateSlow();
    void checkBallBehindKeeper();
    void clipTarget();
    void checkExtendKeeperFrame();
    void checkDisableObstacleAvoidance();
    
    // data members
    actionResultTypeEnum _result;
    Position2D _currentPos;
    Position2D _targetPos;
    motionTypeEnum _motionType = motionTypeEnum::INVALID;
    bool       _haveKeeperFrame = false;
    bool       _disabledObstacleAvoidance = false;
    
    float _Y_MAX_OFFSET_KEEPER = 0.0;
    float _GOALPOST_OFFSET_KEEPER = 0.0;
    float _FRAME_EXTENSION_SPEED_THRESHOLD = 3.0;
    float _FRAME_SIDE_EXTENSION_TOLERANCE = 0.2;
    float _FRAME_TOP_EXTENSION_TOLERANCE = 0.0;

};

#endif /* MP_ACTIONKEEPERMOVE_HPP_ */

