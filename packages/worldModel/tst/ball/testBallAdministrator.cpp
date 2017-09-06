 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testBallAdministrator.cpp
 *
 *  Created on: Sep 07, 2016
 *      Author: Jan Feitsma
 */

#include <gtest/gtest.h>

#include "int/administrators/ballAdministrator.hpp"

#include "FalconsCommon.h"

TEST(testBallAdministrator, performance)
{
	/*
	 * Setup
	 */
	ballAdministrator adm;

	/*
	 * Execution
	 */

    // assume 30 Hz and 2sec timeout, data coming in from 4 robots
    // we generate the measurements and feed them to the administrator in 1 simulated second
    std::vector<ballMeasurementType> measurements;
    std::vector<ballClass_t> balls;
    float freq = 30.0;
    float tmax = 2.0;
    float dt = 1.0 / freq;
    int N = int(freq * tmax);
    TRACE("start");
    for (int it = 0; it <= N; ++it)
    {
        // add new measurements for this timestamp
        ballMeasurementType c;
        c.setTimestamp(dt * it);
        for (int irobot = 1; irobot <= 4; ++irobot)
        {
            c.setID(uniqueWorldModelID(irobot, it));
            measurements.push_back(c);
        }
        // feed measurements to the administrator
        TRACE("it=%d  size=%d", it, measurements.size());
		adm.appendBallMeasurements(measurements);

	}
    TRACE("end"); // tracing shows this takes only a fraction of a second --> OK
    
	/*
	 * Verification
	 */
	
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
