// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotLocationType.hpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTLOCATIONTYPE_HPP_
#define ROBOTLOCATIONTYPE_HPP_

class robotLocationType
{
	public:
		robotLocationType();
		~robotLocationType();

		void setX(const float x);
		void setY(const float y);
		void setTheta(const float theta);
		void setConfidence(const float confidence);
		void setFPS(const float fps);
		void setLinePoints(const int linePoints);
		void setAge(const float age);
		void setLastActive(const float lastActive);

		float getX() const;
		float getY() const;
		float getTheta() const;
		float getConfidence() const;
		float getFPS() const;
		int getLinePoints() const;
		float getAge() const;
		float lastActive() const;

	private:
		float _x;
		float _y;
		float _theta;
		float _confidence;
		float _fps;
		int _linePoints;
		float _age;
		float _lastActive;
};






#endif /* ROBOTLOCATIONTYPE_HPP_ */
