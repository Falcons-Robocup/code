// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * KeeperFrame.hpp
 *
 *  Created on: Mar 26, 2019
 *      Author: Jan Feitsma
 */

#ifndef INCLUDE_INT_KEEPER_FRAME_HPP_
#define INCLUDE_INT_KEEPER_FRAME_HPP_

#include "int/ioBoard/IoBoard.hpp"
#include "FalconsRtDB2.hpp" // rtime


// only allow a setpoint once every x seconds 
// to avoid spam / erratic behavior
#define KEEPERFRAME_SETPOINT_PERIOD (5.0)


class KeeperFrame {
public:
    KeeperFrame(IoBoard &ioBoard);
    ~KeeperFrame();

    void setKeeperFrameLeft();
    void setKeeperFrameRight();
    void setKeeperFrameUp();

private:
    IoBoard &ioBoard;
    rtime _lastSetpointTime;
    
    bool allowNewSetpoint();
};

#endif

