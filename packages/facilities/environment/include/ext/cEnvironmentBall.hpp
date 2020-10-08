 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cEnvironmentBall.hpp
 *
 *  Created on: Jan 3, 2016
 *      Author: Michel Koenen
 */



#ifndef CENVIRONMENTBALL_HPP_
#define CENVIRONMENTBALL_HPP_


class cEnvironmentBall
{
	public:
			static cEnvironmentBall& getInstance()
			{
				static cEnvironmentBall instance; // Guaranteed to be destroyed.
											          // Instantiated on first use.
				return instance;
			}

			/*
			 * Getter functionality for use by anybody who needs to know
			 */
			float getRadius();


	private:
			cEnvironmentBall();  //constructor definition
			~cEnvironmentBall();
			cEnvironmentBall(cEnvironmentBall const&); // Don't Implement
			void operator=(cEnvironmentBall const&);	   // Don't implement
			void getConfig();
			float _radius;
};

#endif /* CCENVIRONMENTBALL_HPP_ */

