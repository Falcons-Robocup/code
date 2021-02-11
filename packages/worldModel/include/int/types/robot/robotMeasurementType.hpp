// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotType.hpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTMEASUREMENTTYPE_HPP_
#define ROBOTMEASUREMENTTYPE_HPP_

#include "uniqueObjectID.hpp"
#include "int/types/coordinateType.hpp"
#include "position2d.hpp"


class robotMeasurementClass_t
// TODO: Jan2Tim: earlier we renamed ballCandidate to ballMeasurement; but here the situation is different.
// I think current name is a bit ambiguous (what kind of measurement?).
// From vision point of view, a number of candidate locations is provided to worldModel. 
// A considerable amount of processing was done in vision to process measured pixels into candidate.
// Then worldModel must process / choose / make sense of the candidates. 
// Perhaps rename something like robotMeasurementClass_t => robotLocationVisionCandidate_t ?
{
	public:
		robotMeasurementClass_t();
		~robotMeasurementClass_t();

		void setID(const uniqueObjectID identifier);
		void setCoordinateType(const coordinateType coordinates); // unused
		void setTimestamp(const double timestamp);
		void setConfidence(const float confidence);
		void setPosition(const float x, const float y, const float theta);

		uniqueObjectID getID() const;
		coordinateType getCoordindateType() const;
		double getTimestamp() const;
		float getConfidence() const;
		float getX() const;
		float getY() const;
		float getTheta() const;
        Position2D getPosition() const;

	private:
		uniqueObjectID _identifier;
		coordinateType _coordinate;
		double _timestamp;
		float _confidence;
		float _x;
		float _y;
		float _theta;
};

#endif /* ROBOTMEASUREMENTTYPE_HPP_ */
