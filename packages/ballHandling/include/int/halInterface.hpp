 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

