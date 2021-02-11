// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

    vt_motors_data feedbackMotors;
    _vtMain->_vtDataClass->getFeedbackMotorsData(feedbackMotors);

    boost::numeric::ublas::matrix<double> inverseMatrix;
    _vtMain->_vtDataClass->getInvMatrix(inverseMatrix);


    TRACE("") << "Feedback: m1.disp=" << std::to_string(feedbackMotors.m1.displacement) << "; m2.disp=" << std::to_string(feedbackMotors.m2.displacement) << "; m3.disp=" << std::to_string(feedbackMotors.m3.displacement);


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

	vt_robot_data feedbackRobotData;

	feedbackRobotData.displacement.x = vDisplacement(0);
	feedbackRobotData.displacement.y = vDisplacement(1);
	feedbackRobotData.displacement.phi = vDisplacement(2);

	feedbackRobotData.velocity.x = vVelocity(0);
	feedbackRobotData.velocity.y = vVelocity(1);
	feedbackRobotData.velocity.phi = vVelocity(2);

	_vtMain->_vtDataClass->setFeedbackRobotData(feedbackRobotData);

    TRACE("") << "Feedback: x.disp=" << std::to_string(feedbackRobotData.displacement.x) << "; y.disp=" << std::to_string(feedbackRobotData.displacement.y) << "; Rz.disp=" << std::to_string(feedbackRobotData.displacement.phi);

	/* START PTRACE SETGET */

	vt_motors_data targetMotors;
	_vtMain->_vtDataClass->getTargetMotorsData(targetMotors);

	vt_robot_data targetRobotData;
	_vtMain->_vtDataClass->getTargetRobotData(targetRobotData);

//	PTRACE("KST %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f",
//			targetMotors.m1.velocity, targetMotors.m2.velocity, targetMotors.m3.velocity,
//			feedbackMotors.m1.velocity, feedbackMotors.m2.velocity, feedbackMotors.m3.velocity,
//			targetRobotData.velocity.x, targetRobotData.velocity.y, targetRobotData.velocity.phi,
//			feedbackRobotData.velocity.x, feedbackRobotData.velocity.y, feedbackRobotData.velocity.phi);

	/* END PTRACE SETGET */

}


