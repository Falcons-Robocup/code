 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * packetConstructorWorldModel.hpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#ifndef PACKETCONSTRUCTORWORLDMODEL_HPP_
#define PACKETCONSTRUCTORWORLDMODEL_HPP_

#include <vector>

#include "int/types/packetStructureWorldModel.hpp"

#include "cByteArray.hpp"

class packetConstructorWorldModel
{
	public:
		packetConstructorWorldModel();
		~packetConstructorWorldModel();

		void setPacketCounter(const uint8_t value);
		void setPacketVersion(const uint8_t value);
		void setGroupID(const uint8_t value);
		void setRobotID(const uint8_t value);
		void setBallMeasurements(std::vector<ballMeasurement> measurements);
		void setBallPossession(const bool hasBallPossession);
		void setObstacleMeasurements(std::vector<obstacleMeasurement> measurements);
		void setRobotLocation(const robotLocationStructure location);
		void setByteArray(Facilities::cByteArray byteArray);

		Facilities::cByteArray getByteArray();
		packetStructureWorldModel getWorldModelStructure() const;

	private:
		packetStructureWorldModel _wmPacket;
};

#endif /* PACKETCONSTRUCTORWORLDMODEL_HPP_ */
