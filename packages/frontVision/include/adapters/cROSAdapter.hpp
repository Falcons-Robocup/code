 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cROSAdapter.hpp
 *
 *  Created on: May 2, 2016
 *      Author: Tim Kouters
 */

#ifndef CROSADAPTER_HPP_
#define CROSADAPTER_HPP_

#include <ros/node_handle.h>
#include "boost/thread/mutex.hpp"
#include <math.h>
#include <vector>

#include "linearTable.hpp"
#include "cDiagAdapter.hpp"
#include "types/ballPositionType.hpp"


class cROSAdapter
{
	public:
		cROSAdapter(cDiagAdapter *);
		~cROSAdapter();

		void initializeAdapter();
		void setBall(const std::vector<ballPositionType> &balls);

	private:
		const float ANGLE_OFFSET = M_PI_2;
		const float CAMERA_MOUNTING_HEIGHT = 0.38;
		const float CAMERA_FRONT_OFFSET = 0.15;
		const float CAMERA_FRONT_PHI = M_PI_2;

        boost::shared_ptr<ros::NodeHandle> _hROS;
		ros::ServiceClient _sSetFrontCamera;
		lookup::unbounded_lookup1d _lutDistance;
		cDiagAdapter *_diagAdapter;
};

#endif /* CROSADAPTER_HPP_ */
