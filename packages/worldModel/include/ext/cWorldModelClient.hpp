 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelClient.hpp
 *
 * WorldModel client facility, intended for motionPlanning, teamplay, pathPlanning
 * Aiming to minimize type/code duplication while hiding RTDB design choice from clients.
 * Takes care of RTDB interfacing; provides data using sharedTypes, as well as a set of convenience functions / type adapters.
 *
 *  Created on: Aug 18, 2018
 *      Author: Jan Feitsma
 */

#ifndef CWORLDMODELCLIENT_HPP_
#define CWORLDMODELCLIENT_HPP_

// system includes
#include <vector>
#include <map>

// WM config
#include "RTDBConfigAdapter.hpp" // TODO: not so nice to make this externally visible ..

// RTDB
#include "cRtDBClient.hpp"
#include "FalconsRtDB2.hpp"

#include "falconsCommon.hpp" // TODO fix type dealing abuse, use geometry package

// defines
#define ACTIVE_TIMEOUT 7.0 // seconds - ignore teammember if data is older


class cWorldModelClient : public cRtDBClient
{
public:
    cWorldModelClient();
    ~cWorldModelClient();
    
    // update to GET all data from RTDB, store for all getters
    void update();
    void update(const int myRobotId);
    
    // getters
    bool isActive() const;
    Position2D getPosition() const;
    Velocity2D getVelocity() const;
    bool noBall() const;
    bool hasBall() const;
    bool teamHasBall() const;
    bool opponentHasBall() const;
    Vector3D ballPosition() const;
    Vector3D ballVelocity() const;
    T_BALLS getBalls() const;
    T_OBSTACLES getObstacles() const;
    int numObstacles() const;
    float closestObstacleDistance() const;
    bool getRobotState(T_ROBOT_STATE &robot, const int robotId); // return success
    bool getRobotState(T_ROBOT_STATE &robot, const int robotId, const int myRobotId);
    std::vector<T_ROBOT_STATE> getTeamMembersExcludingSelf() const;
    // TODO estimated ball direction (if opponent seems to have the ball)
    // TODO last known ball location
    
private:
    // world state data
    teamIdType _myTeamId = "";
    T_ROBOT_STATE _robotState;
    std::map<int, T_ROBOT_STATE> _teamState;
    T_BALLS _balls;
    T_OBSTACLES _obstacles;
    RTDBConfigAdapter _configAdapter;
    T_CONFIG_WORLDMODELSYNC _config;
};

#endif

