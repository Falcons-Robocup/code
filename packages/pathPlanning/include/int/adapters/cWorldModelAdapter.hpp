 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelAdapter.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#ifndef CWORLDMODELADAPTER_HPP_
#define CWORLDMODELADAPTER_HPP_

#include <stdio.h>

#include "ros/node_handle.h"
#include "ros/service.h"
#include "ros/timer.h"
#include "boost/thread/mutex.hpp"
#include "worldModel/t_wmInfo.h"
#include "WorldModelNames.h"
#include "linepoint2D.hpp"

#include "int/cPathPlanningTypes.hpp"
#include "int/cPathPlanningData.hpp"

/*!
 * \brief The ROS adapter to WorldModel
 *
 * This adapter is the ROS adapter to WorldModel.
 * Every fetch() will do a ROS call to WorldModel and store the data in cPathPlanningData.
 * A callback will set the data in cPathPlanningData and return (optionally calling iterateFunction).
 */
class cWorldModelAdapter
{

    public:
        cWorldModelAdapter();
        cWorldModelAdapter(cPathPlanningData &data, iterateFunctionType func);
        ~cWorldModelAdapter();

        void initializeWM();
    private:
        boost::shared_ptr<ros::NodeHandle> _hROS;
        ros::Subscriber _subWmInfo;

        uint8_t _robotID;

        iterateFunctionType _iterateFunc;
        cPathPlanningData* _ppData;

        void worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg);

        void fetchPositionAndVelocity(const worldModel::t_wmInfo::ConstPtr& msg);
        void fetchObstacles(const worldModel::t_wmInfo::ConstPtr& msg);
        void fetchRobotActive(const worldModel::t_wmInfo::ConstPtr& msg);
        void fetchBall(const worldModel::t_wmInfo::ConstPtr& msg);
        void fetchOwnRobotLowestActive(const worldModel::t_wmInfo::ConstPtr& msg);

        void projectObstacles(const pp_obstacle_struct_t& obst, const pp_limiters_struct_t& limits, std::vector<pp_obstacle_struct_t>& obstacles, std::vector<linepoint2D>& projectedSpeedPoint);
};






#endif /* CWORLDMODELADAPTER_HPP_ */
