 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include "int/types/robot/robotMeasurementType.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/robot/robotStatusType.hpp"
#include "int/algorithms/robotLocalization.hpp"

class robotAdministrator
{
    public:
        robotAdministrator();
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
        robotLocalization _localizationAlgorithm;

        virtual void removeTimedoutRobots(rtime const timeNow);
};

#endif /* ROBOTADMINISTRATOR_HPP_ */
