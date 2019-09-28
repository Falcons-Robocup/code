 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleAdministratorMock.hpp
 *
 *  Created on: Nov 25, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLEADMINISTRATORMOCK_HPP_
#define OBSTACLEADMINISTRATORMOCK_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/administrators/obstacleAdministrator.hpp"

class obstacleAdministratorMock : public obstacleAdministrator
{
    public:
    	MOCK_METHOD1(appendObstacleMeasurements, void(const T_OBSTACLE_CANDIDATES measurements));
    	MOCK_METHOD1(overruleObstacles, void(const std::vector<obstacleClass_t> obstacles));
    	MOCK_METHOD1(getLocalObstacleMeasurements, void(T_OBSTACLE_CANDIDATES &measurements));
    	MOCK_METHOD1(performCalculation, void(rtime const timeNow));
    	MOCK_METHOD1(getObstacles, void(std::vector<obstacleClass_t> &balls));

    private:
    	MOCK_METHOD1(cleanUpTimedOutObstacleMeasurements, void(rtime const timeNow));
};




#endif /* OBSTACLEADMINISTRATORMOCK_HPP_ */
