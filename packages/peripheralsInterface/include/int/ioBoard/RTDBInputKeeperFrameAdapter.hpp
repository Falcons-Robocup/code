// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBInputKeeperFrameAdapter.hpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#ifndef INCLUDE_INT_ADAPTERS_RTDBINPUTKEERERFRAMEADAPTER_HPP_
#define INCLUDE_INT_ADAPTERS_RTDBINPUTKEERERFRAMEADAPTER_HPP_

#include "int/ioBoard/KeeperFrame.hpp"
#include "FalconsRtDB2.hpp"

class RTDBInputKeeperFrameAdapter 
{
public:
    RTDBInputKeeperFrameAdapter(KeeperFrame& kf);
    ~RTDBInputKeeperFrameAdapter();

    void loop();
    void process();

private:
    KeeperFrame &_keeperFrame;
    int _myRobotId;
    RtDB2 *_rtdb;

};

#endif

