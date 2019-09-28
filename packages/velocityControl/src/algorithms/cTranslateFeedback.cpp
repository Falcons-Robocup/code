 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTranslateFeedback.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/algorithms/cTranslateFeedback.hpp"

void cTranslateFeedback::execute()
{
    TRACE_FUNCTION("");

    vc_motors_data feedbackMotors;
    _vcMain->_vcDataClass->getFeedbackMotorsData(feedbackMotors);

    boost::numeric::ublas::matrix<double> inverseMatrix;
    _vcMain->_vcDataClass->getInvMatrix(inverseMatrix);




    /*
     * ==> Translate FEEDBACK motor velocity to robot velocity
     */
	boost::numeric::ublas::vector<double> mVelocity(3);
	mVelocity(0) = feedbackMotors.m1.velocity;
	mVelocity(1) = feedbackMotors.m2.velocity;
	mVelocity(2) = feedbackMotors.m3.velocity;

	boost::numeric::ublas::vector<double> vVelocity = boost::numeric::ublas::prod(inverseMatrix, mVelocity);

	boost::numeric::ublas::vector<double> mDisplacement(3);
	mDisplacement(0) = feedbackMotors.m1.displacement;
	mDisplacement(1) = feedbackMotors.m2.displacement;
	mDisplacement(2) = feedbackMotors.m3.displacement;

	boost::numeric::ublas::vector<double> vDisplacement = boost::numeric::ublas::prod(inverseMatrix, mDisplacement);

	vc_robot_data feedbackRobotData;

	feedbackRobotData.displacement.x = vDisplacement(0);
	feedbackRobotData.displacement.y = vDisplacement(1);
	feedbackRobotData.displacement.phi = vDisplacement(2);

	feedbackRobotData.velocity.x = vVelocity(0);
	feedbackRobotData.velocity.y = vVelocity(1);
	feedbackRobotData.velocity.phi = vVelocity(2);

	_vcMain->_vcDataClass->setFeedbackRobotData(feedbackRobotData);

	/* START PTRACE SETGET */

	vc_motors_data targetMotors;
	_vcMain->_vcDataClass->getTargetMotorsData(targetMotors);

	vc_robot_data targetRobotData;
	_vcMain->_vcDataClass->getTargetRobotData(targetRobotData);

//	PTRACE("KST %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f",
//			targetMotors.m1.velocity, targetMotors.m2.velocity, targetMotors.m3.velocity,
//			feedbackMotors.m1.velocity, feedbackMotors.m2.velocity, feedbackMotors.m3.velocity,
//			targetRobotData.velocity.x, targetRobotData.velocity.y, targetRobotData.velocity.phi,
//			feedbackRobotData.velocity.x, feedbackRobotData.velocity.y, feedbackRobotData.velocity.phi);

	/* END PTRACE SETGET */

}


