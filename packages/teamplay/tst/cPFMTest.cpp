 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPFMTest.cpp
 *
 *  Created on: May 22, 2016
 *      Author: Tim Kouters
 */
#include <gtest/gtest.h>
#include "int/utilities/trace.hpp"

#include "int/algorithms/cPFM.hpp"

TEST(PFMTester, testDump)
{
	/*
	 * Setup
	 */
	Area2D area = Area2D(-2.0 , -2.0, 2.0, 2.0);
	cPFM pfm = cPFM(area);

	std::vector<Position2D> obstacles;
	obstacles.push_back(Position2D(-1.0, -1.0, 0.0));
	obstacles.push_back(Position2D(1.0, 1.0, 0.0));
	pfm.setObstacles(obstacles);

	pfm.setTargetPosition(Position2D(2.0, 2.0, 0.0));
	/*
	 * Execution
	 */
	Position2D ownPos = Position2D(-1.0, -1.0, 0.0);
	Position2D bestPos;
	pfm.getOptimalFreePosition(ownPos, bestPos, true);

	/*
	 * Verification
	 */
	std::cout << "best pos: " << bestPos.tostr() << std::endl;

}


// MAIN
int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
