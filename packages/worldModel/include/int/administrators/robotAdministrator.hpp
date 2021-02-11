// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotAdministrator.hpp
 *
 *  Created on: Aug 18, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTADMINISTRATOR_HPP_
#define ROBOTADMINISTRATOR_HPP_

#include <stdint.h>
#include <map>
#include <vector>

#include "diagWorldModel.hpp"

#include "int/adapters/configurators/WorldModelConfig.hpp"

#include "int/types/robot/robotMeasurementType.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/robot/robotStatusType.hpp"
#include "int/algorithms/robotLocalization.hpp"

class robotAdministrator
{
    public:
        robotAdministrator(WorldModelConfig& wmConfig);
        virtual ~robotAdministrator();

        virtual void appendRobotVisionMeasurements(const std::vector<robotMeasurementClass_t> measurements);
        virtual void appendRobotDisplacementMeasurements(const std::vector<robotDisplacementClass_t> displacements);
        virtual void appendRobotVelocityMeasurements(const std::vector<robotVelocityClass_t> velocities);
        virtual void updateRobotPositionAndVelocity(const robotClass_t robot);
        virtual void disableOverrulingOfLocalRobot();
        virtual void setRobotStatus(const uint8_t robotID, const robotStatusType status, rtime const timeNow);

        virtual void performCalculation(rtime const timeNow);
        bool inplay();
        void determineInPlay();
        void fillDiagnostics(diagWorldModel &diagnostics);
        
        void setBallPossessionBallHandlers(bool enabled) { _ballPossessionBallHandlers = enabled; }
        void setBallPossessionVision(bool enabled) { _ballPossessionVision = enabled; }
        void calcBallPossession(double timestamp);
        bool getBallPossession() const;
        void getBallClaimedPosition(float &x, float &y);

        virtual robotClass_t getLocalRobotPosition(double timestamp = 0.0);
        virtual std::vector<robotClass_t> getTeammembers();
        virtual std::vector<uint8_t> getActiveMembers();

    private:
        uint8_t _ownRobotID;
        bool _isLocationValid;
        bool _isSimulated;
        bool _wasInplay;
        bool _inplay;
        bool _ballPossessionBallHandlers;
        bool _ballPossessionVision;
        bool _ballPossession;
        bool _prevBallPossession;
        float _ballClaimedX, _ballClaimedY;
        std::map<uint8_t, robotClass_t> _robots;
        std::map<uint8_t, robotStatusType> _robotStatus;

        virtual void removeTimedoutRobots(rtime const timeNow);

        WorldModelConfig& _wmConfig;
        robotLocalization _localizationAlgorithm;
};

#endif /* ROBOTADMINISTRATOR_HPP_ */
