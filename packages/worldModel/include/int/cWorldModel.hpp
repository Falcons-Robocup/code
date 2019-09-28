 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModel.hpp
 *
 *  Created on: January, 2019
 *      Author: Jan Feitsma
 */

#ifndef CWORLDMODEL_HPP_
#define CWORLDMODEL_HPP_


#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"
#include "int/adapters/adaptersCollector.hpp"
#include "int/adapters/ROS/heartBeatAdapterROS.hpp"
#include "int/adapters/RTDB/RTDBHeartBeatAdapter.hpp"

#include "int/adapters/RTDB/RTDBInputAdapter.hpp"

#include "int/adapters/configurators/administratorConfigROS.hpp"
#include "int/adapters/configurators/ballTrackerConfigROS.hpp"
#include "int/adapters/configurators/localizationConfigROS.hpp"
#include "int/adapters/configurators/obstacleTrackerConfigROS.hpp"
#include "ext/WorldModelNames.h"

#include "cDiagnostics.hpp"



class cWorldModel
{
public:
    cWorldModel();
    ~cWorldModel();
    
    void updateNow(bool dummy); // use current timestamp
    void update(rtime const timeNow); // use provided timestamp

    diagWorldModel getDiagnostics();
    
private:
    void initialize();
    
    // administrators
    robotAdministrator       _robotAdmin;
    ballAdministrator        _ballAdmin;
    obstacleAdministrator    _obstacleAdmin;

    // adapters
    adaptersCollector        _adpCollector;
    RTDBInputAdapter         _rtdbInput;
    RTDBOutputAdapter        _rtdbOutput;
    RTDBHeartBeatAdapter     _adpHeartBeatRTDB;

    // create ROS & reconfigure adapters (to be migrated to RTDB)
    heartBeatAdapterROS      _adpHeartBeatROS; // still needed for coach
    administratorConfigROS   _adpAdminConfigROS;
    ballTrackerConfigROS     _adpBallTrackerConfigROS;
    localizationConfigROS    _adpLocalizationConfigROS;
    obstacleTrackerConfigROS _adpObstacleTrackerConfigROS;

};

#endif

