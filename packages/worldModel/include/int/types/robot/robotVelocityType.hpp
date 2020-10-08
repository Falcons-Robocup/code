 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
