 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballAdministrator.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLADMINISTRATOR_HPP_
#define BALLADMINISTRATOR_HPP_

#include <stdint.h>
#include <map>
#include <vector>

#include "int/types/ball/ballMeasurementType.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/administrators/ballDiscriminator.hpp"
#include "int/types/uniqueWorldModelIDtype.hpp"

class ballAdministrator
{
	public:
		ballAdministrator();
		virtual ~ballAdministrator();

		virtual void appendBallMeasurements(const std::vector<ballMeasurementType> measurements);
		virtual void overruleBall(const ballClass_t ball);
		virtual void getLocalBallMeasurements(std::vector<ballMeasurementType> &measurements);
		virtual void performCalculation(const double timeNow);

		virtual void getOwnBalls(std::vector<ballClass_t> &balls);
		virtual void getGlobalBalls(std::vector<ballClass_t> &balls);

	private:
		uint8_t _ownRobotID;
		std::map<uint8_t, ballClass_t> _overruledBalls;
		std::map<uniqueWorldModelID, ballMeasurementType> _ballMeasurements;

		ballDiscriminator _globalBallDiscriminator;
		ballDiscriminator _localBallDiscriminator;

		virtual void cleanUpTimedOutBallMeasurements(const double timeNow);
};

#endif /* BALLADMINISTRATOR_HPP_ */
