 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
