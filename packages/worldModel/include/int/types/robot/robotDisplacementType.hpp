// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotDisplacementType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTDISPLACEMENTTYPE_HPP_
#define ROBOTDISPLACEMENTTYPE_HPP_

#include "uniqueObjectID.hpp"
#include "int/types/coordinateType.hpp"
#include "int/types/displacementType.hpp"

class robotDisplacementClass_t
{
	public:
		robotDisplacementClass_t();
		~robotDisplacementClass_t();

		void setID(const uniqueObjectID identifier);
		void setCoordinateType(const coordinateType coordinates);
		void setDisplacementSource(const displacementType displacementSource);
		void setTimestamp(const double timestamp);
		// NOTE: it is assumed that consistent position/velocity is provided (by peripheralsInterface)
		void setDeltaPosition(const float dx, const float dy, const float dtheta);

		uniqueObjectID getID() const;
		coordinateType getCoordinateType() const;
		displacementType getDisplacementSource() const;
		double getTimestamp() const;
		float getdX() const;
		float getdY() const;
		float getdTheta() const;

	private:
		uniqueObjectID _identifier;
		coordinateType _coordinate;
		displacementType _displacementSource;
		double _timestamp;
		float _dx;
		float _dy;
		float _dtheta;
};

#endif /* ROBOTDISPLACEMENTTYPE_HPP_ */
