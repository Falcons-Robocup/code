// Copyright 2018 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotVelocityType.hpp
 *
 *  Created on: Nov 8, 2018
 *      Author: Erik Kouters
 */

#ifndef ROBOTVELOCITYTYPE_HPP_
#define ROBOTVELOCITYTYPE_HPP_

#include "uniqueObjectID.hpp"
#include "int/types/coordinateType.hpp"
#include "int/types/displacementType.hpp"

class robotVelocityClass_t
{
	public:
        robotVelocityClass_t();
		~robotVelocityClass_t();

		void setID(const uniqueObjectID identifier);
		void setCoordinateType(const coordinateType coordinates);
		void setDisplacementSource(const displacementType displacementSource);
		void setTimestamp(const double timestamp);
		// NOTE: it is assumed that consistent position/velocity is provided (by peripheralsInterface)
		void setDeltaVelocity(const float vx, const float vy, const float vtheta);

		uniqueObjectID getID() const;
		coordinateType getCoordinateType() const; // TODO: fix typo (impact on many files)
		displacementType getDisplacementSource() const;
		double getTimestamp() const;
		float getvX() const;
		float getvY() const;
		float getvTheta() const;

	private:
		uniqueObjectID _identifier;
		coordinateType _coordinate;
		displacementType _displacementSource;
		double _timestamp;
		float _vx;
		float _vy;
		float _vtheta;
};

#endif /* ROBOTVELOCITYTYPE_HPP_ */
