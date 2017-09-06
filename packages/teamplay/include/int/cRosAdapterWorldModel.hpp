 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterWorldModel.hpp
 *
 *  Created on: Oct 7, 2015
 *      Author: Jan Feitsma
 */

#ifndef CROSADAPTERWORLDMODEL_HPP_
#define CROSADAPTERWORLDMODEL_HPP_

#include "ros/ros.h"

#include <map>
#include <string>
#include "position2d.hpp"

#include "worldModel/t_wmInfo.h"

/*!
 * \brief The ROS adapter class to get the WorldModel inputs for Teamplay
 *
 * The class cRosAdapterWorldModel is a singleton class which calls the getters to 
 * WorldModel and stores the result in cWorldModelInterface by calling its setters.
 */
class cRosAdapterWorldModel
{
   public:
	/*!
	 * \brief Obtain an instance of this singleton class
	 */
	static cRosAdapterWorldModel& getInstance()
	{
		static cRosAdapterWorldModel instance; 
		return instance;
	}

	/*!
	 * \brief Do a round of getters to WorldModel and store the results
	 */
	void update(const worldModel::t_wmInfo::ConstPtr& msg);

   private:
	cRosAdapterWorldModel();
	ros::NodeHandle _hROS;
	ros::Subscriber _wmInfo;
	ros::NodeHandle _nh;
	std::map<std::string, ros::ServiceClient> _serviceSubscriptions;

	void updateOwnLocation(const worldModel::t_wmInfo::ConstPtr& msg);
	void updateTeammembers(const worldModel::t_wmInfo::ConstPtr& msg);
	void updateOpponents(const worldModel::t_wmInfo::ConstPtr& msg);
	void updateBallLocation(const worldModel::t_wmInfo::ConstPtr& msg);
	void updateBallPossession(const worldModel::t_wmInfo::ConstPtr& msg);

	// TODO factor out parts to a base class cRosAbstractAdapter

	template <typename T>
	void registerService(std::string const &servicename)
	{
	        // check that key is not used
		assert(!_serviceSubscriptions.count(servicename));
		// wait for service to come online
		ros::service::waitForService(servicename);
		// subscribe and register the input service (use persistency flag)
		_serviceSubscriptions[servicename] = _nh.serviceClient<T>(servicename, true);
	}
}; // class cRosAdapterWorldModel

#endif /* CROSADAPTERWORLDMODEL_HPP_ */
