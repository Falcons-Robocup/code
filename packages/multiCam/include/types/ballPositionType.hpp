// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballPositionType.hpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLPOSITIONTYPE_HPP_
#define BALLPOSITIONTYPE_HPP_

#include <stddef.h>

class ballPositionType
{
	public:
		ballPositionType();
		~ballPositionType();

		void setAngle(const float angle);
		void setRadius(const float radius);
		void setElevation(const float elevation);
		void setConfidence(const float confidence);
		void setBallType(size_t type);

		float getAngle() const;
		float getRadius() const;
		float getElevation() const;
		float getConfidence() const;
		size_t getBallType() const;

	private:
		float _angle;
		float _radius;
		float _elevation;
		float _confidence;
		size_t _type;
};

#endif /* BALLPOSITIONTYPE_HPP_ */
