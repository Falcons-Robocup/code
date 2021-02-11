// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * halInterface.hpp
 *
 * Exchange ballHandler data.
 * Get the status (angle) and send the velocity setpoints.
 * 
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#ifndef HALINTERFACE_HPP_
#define HALINTERFACE_HPP_

#include <functional>

#include "int/types/ballHandlersStatusType.hpp"
#include "int/types/ballHandlersSetpointsType.hpp"
#include "int/types/ballHandlerControlModeType.hpp"

class halInterface
{
	public:
		using ballHandlersStatusCallback = std::function<void(ballHandlersStatusType)>;

		halInterface() { _setBallhandlersStatus = NULL; };
		virtual ~halInterface() {};

	    virtual ballHandlersStatusType getBallHandlersStatus() = 0;
		virtual void setBallHandlersSetpoints(ballHandlersSetpointsType const &b) = 0;
		virtual void setBallHandlerControlMode(ballHandlerControlModeType controlMode) = 0;

		virtual void subscribeBallHandlersStatusCallback(ballHandlersStatusCallback callback) { _setBallhandlersStatus = callback; };

	protected:
		ballHandlersStatusCallback _setBallhandlersStatus;
};

#endif /* HALINTERFACE_HPP_ */

