// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * controlInterface.hpp
 *
 * During a match, the only required interface is to disable/enable ballHandlers.
 * This is controlled by motionPlanning.
 * The ballHandlingControl algorithm is expected to actively read the enabled/disabled status.
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#ifndef CONTROLINTERFACE_HPP_
#define CONTROLINTERFACE_HPP_

#include <functional>

class controlInterface
{
  public:
	using controlActionCallback = std::function<void()>;

    controlInterface() {};
    virtual ~controlInterface() {};

	virtual void subscribeEnableBallhandlersCallback(controlActionCallback callback) { _enableBallhandlers = callback; };
	virtual void subscribeDisableBallhandlersCallback(controlActionCallback callback) { _disableBallhandlers = callback; };

  protected:
	controlActionCallback _enableBallhandlers;
	controlActionCallback _disableBallhandlers;
};

#endif /* CONTROLINTERFACE_HPP_ */

