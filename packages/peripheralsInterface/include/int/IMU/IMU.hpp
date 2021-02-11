// Copyright 2016 Edwin (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IMU.hpp
 *
 *  Created on: Feb 23, 2016
 *      Author: Edwin Schreuder
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include "tf/LinearMath/Vector3.h"


#include "int/IMU/Yocto3DModule.hpp"

using tf::Vector3;

class IMU {
	public:
        IMU();

		double getCompassAngle();
		double getRawCompassAngle();

		double getHomeGoalAngle();
		void setHomeGoalAngle(double angle);

		Vector3 getVector();

	private:
		Yocto3DModule _module;

		double _homeGoalAngle;
};

#endif /* IMU_HPP_ */
