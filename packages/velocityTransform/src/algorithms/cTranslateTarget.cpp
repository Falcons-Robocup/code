// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTranslateTarget.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/algorithms/cTranslateTarget.hpp"

void cTranslateTarget::execute()
{
    TRACE_FUNCTION("");

    /*
     * ==> Translate TARGET robot velocity to motor velocity
     */

    vt_robot_data robotData;
    _vtMain->_vtDataClass->getTargetRobotData(robotData);

    boost::numeric::ublas::matrix<double> matrix;
	_vtMain->_vtDataClass->getMatrix(matrix);

	boost::numeric::ublas::vector<double> v(3);
	v(0) = robotData.velocity.x;
	v(1) = robotData.velocity.y;
	v(2) = robotData.velocity.phi;

	boost::numeric::ublas::vector<double> m = boost::numeric::ublas::prod(matrix, v);

	vt_motors_data motorsData;

	motorsData.m1.velocity = m(0);
	motorsData.m2.velocity = m(1);
	motorsData.m3.velocity = m(2);

	_vtMain->_vtDataClass->setTargetMotorsData(motorsData);

}


