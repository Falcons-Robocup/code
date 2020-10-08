 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
