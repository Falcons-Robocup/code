 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
