 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: robocup
 */

#ifndef OBSTACLETYPE_HPP_
#define OBSTACLETYPE_HPP_

#include <stddef.h>
#include "int/types/object/objectResultType.hpp"

class obstacleClass_t: public objectResultType
{
	public:
		obstacleClass_t();
		obstacleClass_t(objectResultType const &o) : objectResultType(o) {};
		~obstacleClass_t();

		/*
		 * Function needed for std::sort to sort vectors
		 */
		bool operator<(const obstacleClass_t& obj) const
		{
			return this->getConfidence() > obj.getConfidence();
		}

		// assignment operator
        obstacleClass_t& operator= (const obstacleClass_t& d)
        {
            objectResultType::operator=(d);
            return *this;
        }

		void setConfidence(const float confidence);
		float getConfidence() const;

	private:
		float _confidence;
		// TODO try to determine obstacle orientation?
};

#endif /* OBSTACLETYPE_HPP_ */
