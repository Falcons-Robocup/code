 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionMoveMock.hpp
 *
 *  Created on: Jan 5, 2016
 *      Author: Tim Kouters
 */

#ifndef TST_MOCKS_CACTIONMOVEMOCK_HPP_
#define TST_MOCKS_CACTIONMOVEMOCK_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/actions/cActionMove.hpp"   // load the abstract interface class definition

/* Unfortunately google mock is not included in ROS
 * Installing it with apt-get install google-mock will install a later version of gmock than the ROS build in gtest
 * This gives compile errors.
 * See:
 * http://answers.ros.org/question/199680/catkin-doesnt-play-nice-with-google-mock/
 * http://stackoverflow.com/questions/22065098/gmock-doesnt-compile-gtest-exclusive-lock-required-seems-to-not-be-defined
 */


class cActionMoveMock : public cActionMove    //implement functions in the interface class
{   // implement ALL virtual functions as defined in the parent class , otherwise this class will stay abstract and cannot be instantiated from!
	public:
		//~cActionMoveMock();
		MOCK_METHOD3(moveTo, void(float x, float y, float phi));
};

#endif /* TST_MOCKS_CACTIONMOVEMOCK_HPP_ */
