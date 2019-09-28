 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 
#ifndef ROS_GAMESIGNALADAPTER_H
#define ROS_GAMESIGNALADAPTER_H



#include <vector>

// ROS:
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

// Internal:
#include "int/GameSignalAdapter.h"

// Falcons shared code:
#include "rosMsgs/t_worldmodel.h"
#include "rosMsgs/t_worldmodel_team.h"
#include "rosMsgs/t_ana_matchstate.h"
#include "rosMsgs/t_ana_online.h"
#include "rosMsgs/t_ana_comm.h"
#include "rosMsgs/t_diag_worldmodel.h"
#include "rosMsgs/t_diag_wm_loc.h"
#include "rosMsgs/t_diag_wm_ball.h"
#include "rosMsgs/t_diag_wm_obstacles.h"
#include "rosMsgs/t_diag_wm_top.h"
#include "rosMsgs/t_diag_control.h"
#include "rosMsgs/t_diag_vision.h"
#include "rosMsgs/t_diag_health_slow.h"
#include "rosMsgs/t_diag_health_mid.h"
#include "rosMsgs/t_diag_health_fast.h"
#include "rosMsgs/t_diag_pathpl.h"
#include "rosMsgs/t_diag_teamplay.h"
#include "rosMsgs/t_diag_error.h" // legacy
#include "rosMsgs/t_diag_info.h" // legacy
#include "rosMsgs/t_event.h"
#include "rosMsgs/t_diag_halmw.h"
#include "rosMsgs/t_diag_frontvision.h"
#include "rosMsgs/t_diag_refbox.h"
#include "rosMsgs/t_diag_vision_v2.h"

/*
* Specific ROS implementation of GameSignalAdapter
*/
class RosGameSignalAdapter : public GameSignalAdapter
{
    Q_OBJECT

public:
    RosGameSignalAdapter();
    ~RosGameSignalAdapter();

private:
    ros::NodeHandle* _rosNode;
    QTimer* _rosSpinTimer;
    double _currentTimestamp;
    std::vector<ros::Subscriber> _rosSubscribers;

    // Subscribe and store subscriber
    template <typename TMessageType, typename TFunctionType>
    void subscribe(std::string message, int bufferSize, TFunctionType function)
    {
        typedef typename TMessageType::ConstPtr Type;
        boost::function<void (const Type&)> robotCallback = boost::bind(function, this, _1); 

        ros::Subscriber sub = _rosNode->subscribe(message, bufferSize, robotCallback);
        _rosSubscribers.push_back(sub);
    }

    // Performs currying by robot id to subscribe the specified function to the specified message
    template <typename TMessageType, typename TFunctionType>
    void subscribe(uint8_t id, std::string message, int bufferSize, TFunctionType function)
    {
        typedef typename TMessageType::ConstPtr Type;
        boost::function<void (const Type&)> robotCallback = boost::bind(function, this, id, _1); 

        ros::Subscriber sub = _rosNode->subscribe(message, bufferSize, robotCallback);
        _rosSubscribers.push_back(sub);
    }
    
    /* == Helpers == */
    
    // convert ROS event message into visualizer internal LogEvent struct
    LogEvent convertRosEvent(const rosMsgs::t_event::ConstPtr& rosEvent);
    double getCurrentTimestamp();
    void setCurrentTimestamp(double t);

    /* == Topic subscription methods == */

    // /teamA/g_ana_*:
    void teamAnalyticsMatchStateCallback(const rosMsgs::t_ana_matchstate::ConstPtr& msg);
    void teamAnalyticsEventCallback(const rosMsgs::t_event::ConstPtr& msg);
    void teamAnalyticsOnlineCallback(const rosMsgs::t_ana_online::ConstPtr& msg);
    void teamWorldModelCallback(const rosMsgs::t_worldmodel_team::ConstPtr& msg);
    void teamWorldModelOldCallback(const rosMsgs::t_worldmodel::ConstPtr& msg); // legacy wmV1

    // /teamA/robotN/g_diag_*:
    void robotWorldModelCallback(uint8_t robotId, const rosMsgs::t_diag_worldmodel::ConstPtr& msg);
    void robotWorldModelLocalizationCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_loc::ConstPtr& msg);
    void robotWorldModelBallTrackingCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_ball::ConstPtr& msg);
    void robotWorldModelObstaclesCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_obstacles::ConstPtr& msg);
    void robotWorldModelTopCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_top::ConstPtr& msg);
    void robotControlCallback(uint8_t robotId, const rosMsgs::t_diag_control::ConstPtr& msg);
    void robotVisionCallback(uint8_t robotId, const rosMsgs::t_diag_vision::ConstPtr& msg);
    void robotVisionV2Callback(uint8_t robotId, const rosMsgs::t_diag_vision_v2::ConstPtr& msg);
    void robotHealthSlowCallback(uint8_t robotId, const rosMsgs::t_diag_health_slow::ConstPtr& msg);
    void robotHealthMidCallback(uint8_t robotId, const rosMsgs::t_diag_health_mid::ConstPtr& msg);
    void robotHealthFastCallback(uint8_t robotId, const rosMsgs::t_diag_health_fast::ConstPtr& msg);
    void robotPathPlanningCallback(uint8_t robotId, const rosMsgs::t_diag_pathpl::ConstPtr& msg);
    void robotTeamPlayCallback(uint8_t robotId, const rosMsgs::t_diag_teamplay::ConstPtr& msg);
    void robotErrorCallback(uint8_t robotId, const rosMsgs::t_diag_error::ConstPtr& msg); // legacy
    void robotInfoCallback(uint8_t robotId, const rosMsgs::t_diag_info::ConstPtr& msg); // legacy
    void robotHalMWCallback(uint8_t robotId, const rosMsgs::t_diag_halmw::ConstPtr& msg);
    void robotFrontVisionCallback(uint8_t robotId, const rosMsgs::t_diag_frontvision::ConstPtr& msg);
    void robotRefboxCallback(uint8_t robotId, const rosMsgs::t_diag_refbox::ConstPtr& msg);

    // /teamA/robotN/g_ana_*:
    void robotAnalyticsCommCallback(uint8_t robotId, const rosMsgs::t_ana_comm::ConstPtr& msg);

private Q_SLOTS:
    void spinOnce();
};

#endif // ROS_GAMESIGNALADAPTER_H
