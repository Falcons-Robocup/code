// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

