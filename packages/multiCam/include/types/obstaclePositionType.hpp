// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstaclePositionType.hpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLEPOSITIONTYPE_HPP_
#define OBSTACLEPOSITIONTYPE_HPP_

class obstaclePositionType
{
	public:
		obstaclePositionType();
		~obstaclePositionType();

		void setAngle(const float angle);
		void setRadius(const float radius);
		void setConfidence(const float confidence);
		void setColor(const int color);

		float getAngle() const;
		float getRadius() const;
		float getConfidence() const;
		int getColor() const;

	private:
		float _angle;
		float _radius;
		float _confidence;
		int _color;
};

#endif /* OBSTACLEPOSITIONTYPE_HPP_ */
