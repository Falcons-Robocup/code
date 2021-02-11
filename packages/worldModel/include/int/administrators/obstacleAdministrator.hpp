// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleAdministrator.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLEADMINISTRATOR_HPP_
#define OBSTACLEADMINISTRATOR_HPP_

#include <stdint.h>
#include <map>
#include <vector>

#include "diagWorldModel.hpp"

#include "int/adapters/configurators/WorldModelConfig.hpp"

#include "int/types/robot/robotType.hpp"
#include "obstacleMeasurement.hpp"
#include "int/administrators/obstacleDiscriminator.hpp"
#include "uniqueObjectID.hpp"


class obstacleAdministrator
{
    public:
        obstacleAdministrator(WorldModelConfig& wmConfig);
        virtual ~obstacleAdministrator();

        virtual void appendObstacleMeasurements(const std::vector<obstacleMeasurement> measurements);
        virtual void overruleObstacles(const std::vector<obstacleClass_t> obstacles);
        virtual void performCalculation(rtime const timeNow);
        virtual void getLocalObstacleMeasurements(std::vector<obstacleMeasurement> &measurements);
        virtual void getObstacles(std::vector<obstacleClass_t> &obstacles);
        virtual void notifyOwnLocation(robotClass_t const &ownLocation);
        virtual void notifyTeamMembers(std::vector<robotClass_t> const &teamMembers);
        
        void fillDiagnostics(diagWorldModel &diagnostics);

    private:
        uint8_t _ownRobotID;
        std::map<uniqueObjectID, obstacleMeasurement> _obstacleMeasurements;
        std::map<uint8_t, std::vector<obstacleClass_t>> _overruledObstacles;
        IobstacleDiscriminator* _obstacleDiscriminator;
        robotClass_t _ownPos;
        std::vector<robotClass_t> _teamMembers;
        std::vector<obstacleClass_t> _resultObstacles;

        WorldModelConfig& _wmConfig;

        virtual void cleanUpTimedOutObstacleMeasurements(rtime const timeNow);

};

#endif /* OBSTACLEADMINISTRATOR_HPP_ */
