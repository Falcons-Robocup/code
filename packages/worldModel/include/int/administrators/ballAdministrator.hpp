 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include "diagWorldModel.hpp"

#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/administrators/ballDiscriminator.hpp"
#include "uniqueObjectID.hpp"

class ballAdministrator
{
    public:
    	ballAdministrator();
    	virtual ~ballAdministrator();

    	virtual void appendBallMeasurements(const std::vector<ballMeasurement> measurements);
    	virtual void overruleBall(const ballClass_t ball);
    	virtual void getLocalBallMeasurements(std::vector<ballMeasurement> &measurements);
    	virtual void performCalculation(rtime const timeNow, Vector2D const &pos);

    	virtual void getBalls(std::vector<ballClass_t> &balls);
        void fillDiagnostics(diagWorldModel &diagnostics);

    private:
    	uint8_t _ownRobotID;
    	std::map<uint8_t, ballClass_t> _overruledBalls;
    	std::map<uniqueObjectID, ballMeasurement> _ballMeasurements;

    	ballDiscriminator _ballDiscriminator;

    	virtual void cleanUpTimedOutBallMeasurements(rtime const timeNow);
};

#endif /* BALLADMINISTRATOR_HPP_ */
