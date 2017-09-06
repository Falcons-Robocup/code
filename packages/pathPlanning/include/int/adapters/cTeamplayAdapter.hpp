 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTeamplayAdapter.hpp
 *
 *  Created on: Sept 06, 2015
 *      Author: Erik Kouters
 */

#ifndef CTEAMPLAYADAPTER_HPP_
#define CTEAMPLAYADAPTER_HPP_

#include <stdlib.h>
#include <ros/ros.h>

#include "FalconsCommon.h"
#include "pathPlanning/s_pathplanning_get_active.h"
#include "pathPlanning/s_pathplanning_set_active.h"
#include "pathPlanning/s_pathplanning_move_while_turning.h"
#include "pathPlanning/s_pathplanning_turn_then_move.h"
#include "pathPlanning/s_pathplanning_move_then_turn.h"
#include "pathPlanning/s_pathplanning_move_at_speed.h"
#include "pathPlanning/s_pathplanning_turn.h"
#include "rosMsgs/t_target.h"

#include "int/cPathPlanningTypes.hpp"
#include "int/cPathPlanningData.hpp"

class cTeamplayAdapter
{
    public:
        cTeamplayAdapter();
        cTeamplayAdapter(cPathPlanningData &data, iterateFunctionType func);
        ~cTeamplayAdapter();

        bool cb_get_active_service(pathPlanning::s_pathplanning_get_active::Request& request, pathPlanning::s_pathplanning_get_active::Response& response);
        bool cb_set_active_service(pathPlanning::s_pathplanning_set_active::Request& request, pathPlanning::s_pathplanning_set_active::Response& response);

        bool cb_move_while_turning_service(pathPlanning::s_pathplanning_move_while_turning::Request& request, pathPlanning::s_pathplanning_move_while_turning::Response& response);
        bool cb_turn_then_move_service(pathPlanning::s_pathplanning_turn_then_move::Request& request, pathPlanning::s_pathplanning_turn_then_move::Response& response);
        bool cb_move_then_turn_service(pathPlanning::s_pathplanning_move_then_turn::Request& request, pathPlanning::s_pathplanning_move_then_turn::Response& response);
        bool cb_move_at_speed_service(pathPlanning::s_pathplanning_move_at_speed::Request& request, pathPlanning::s_pathplanning_move_at_speed::Response& response);
        bool cb_turn_service(pathPlanning::s_pathplanning_turn::Request& request, pathPlanning::s_pathplanning_turn::Response& response);

        void targetCallback(const rosMsgs::t_target::ConstPtr& msg);
        void initializeTP();



    private:
        boost::shared_ptr<ros::NodeHandle> _n;
        ros::Subscriber _subTarget;
        ros::ServiceServer _srvGetActive;
        ros::ServiceServer _srvSetActive;
        ros::ServiceServer _srvMoveWhileTurning;
        ros::ServiceServer _srvTurnThenMove;
        ros::ServiceServer _srvMoveThenTurn;
        ros::ServiceServer _srvMoveAtSpeed;
        ros::ServiceServer _srvTurn;

        iterateFunctionType _iterateFunc;
        cPathPlanningData* _ppData;
};

#endif /* CTEAMPLAYADAPTER_HPP_ */
