 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * observerRos.hpp
 *
 *  Created on: Sep 3, 2014
 *      Author: Tim Kouters
 */

#ifndef OBSERVERROS_HPP_
#define OBSERVERROS_HPP_

#include <stdio.h>
#include "observer.hpp"

#ifndef NOROS
#include "ros/ros.h"
#include "boost/thread/mutex.hpp"
#include "worldModel/set_own_location.h"
#include "worldModel/set_own_ball_location.h"
#include "worldModel/set_own_obstacle_location.h"
#include "worldModel/set_own_camera_ball_possession.h"
#include "rosMsgs/t_diag_vision.h"
#include "cDiagnostics.hpp"
#endif

class observerRos: public observer
{

    public:
        observerRos(const uint robotID, const bool cameraCorrectlyMounted, const float minimumLockTime);
        virtual ~observerRos();

        virtual void update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset);
        virtual void update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset);
        virtual void update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset);
        virtual void update_own_ball_possession(const bool hasPossession);

        void sendDiagnostics();

#ifndef NOROS
        ros::NodeHandle hPosition;
        ros::ServiceClient cPosition;
        ros::ServiceClient cBallPosition;
        ros::ServiceClient cObstaclePosition;
        ros::ServiceClient cBallPossession;
        diagnostics::cDiagnosticsSender<rosMsgs::t_diag_vision> diagSender;
        rosMsgs::t_diag_vision diagData;
        uint robotID;
        worldModel::set_own_location srvRobotPos;
        worldModel::set_own_ball_location srvBallPos;
        std::vector<worldModel::set_own_obstacle_location> srvObstPos;
        worldModel::set_own_camera_ball_possession srvBallPossession;
        boost::mutex mtx;
        float _prev_x;
        float _prev_y;
        float _prev_theta;
#endif

    private:
        bool isSamePosition(const float x, const float y, const float theta);
	void initializeROS();
};

#endif /* OBSERVERROS_HPP_ */
