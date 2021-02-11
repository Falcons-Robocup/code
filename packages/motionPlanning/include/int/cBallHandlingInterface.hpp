// Copyright 2017-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cBallHandlingInterface.hpp
 *
 *  Created on: Nov 21, 2017
 *      Author: Jan Feitsma
 */

#ifndef CBALLHANDLINGINTERFACE_HPP_
#define CBALLHANDLINGINTERFACE_HPP_

class cBallHandlingInterface
{
  public:
    cBallHandlingInterface() {};
    virtual ~cBallHandlingInterface() {};

    virtual void enableBallHandlers() = 0;
    virtual void disableBallHandlers() = 0;

    void tick();
    void suppress(float timeout);
    
  private:
    bool _isSuppressed = false;
    double _enableTimestamp = 0.0;
};

#endif /* CBALLHANDLINGINTERFACE_HPP_ */

