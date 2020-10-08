 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModel.cpp
 *
 *  Created on: January, 2019
 *      Author: Jan Feitsma
 */

#include "int/cWorldModel.hpp"
#include "tracing.hpp"


cWorldModel::cWorldModel()
    : _wmConfig(),
      _robotAdmin(_wmConfig),
      _ballAdmin(_wmConfig),
      _obstacleAdmin(_wmConfig)
{
    initialize();
}

void cWorldModel::initialize()
{
    // initialize adapters
    _adpCollector.initializeRTDB();
    
    // RTDB input adapter
    _rtdbInput.setBallAdministrator(&_ballAdmin);
    _rtdbInput.setObstacleAdministrator(&_obstacleAdmin);
    _rtdbInput.setRobotAdministrator(&_robotAdmin);
    _adpCollector.setRTDBInputAdapter(&_rtdbInput);
    
    // RTDB output adapter
    _rtdbOutput.setBallAdministrator(&_ballAdmin);
    _rtdbOutput.setObstacleAdministrator(&_obstacleAdmin);
    _rtdbOutput.setRobotAdministrator(&_robotAdmin);
    _adpCollector.setRTDBOutputAdapter(&_rtdbOutput);

    // add admins to adapters
    _adpCollector.setBallAdministrator(&_ballAdmin);
    _adpCollector.setObstacleAdministrator(&_obstacleAdmin);
    _adpCollector.setRobotAdministrator(&_robotAdmin);

    // attach adapters
    _adpHeartBeatCoach.setUpdateFunction(boost::bind(&cWorldModel::updateNow, this, _1)); // coach
    _adpHeartBeatRobot.setUpdateFunction(boost::bind(&cWorldModel::updateNow, this, _1)); // robots
}

cWorldModel::~cWorldModel()
{
}

void cWorldModel::run()
{
    // Block on the robot heartbeat (waitForPut)
    _adpHeartBeatRobot.run();
}

void cWorldModel::enableInplayOverrule()
{
    _rtdbInput.enableInplayOverrule();
}

void cWorldModel::updateNow(bool dummy)
{
    update(rtime::now());
}

void cWorldModel::update(rtime const timeNow)
{
    _adpCollector.heartBeatRecalculation(timeNow);
    WRITE_TRACE;
}

diagWorldModel cWorldModel::getDiagnostics()
{
    return _adpCollector.getDiagnostics();
}

