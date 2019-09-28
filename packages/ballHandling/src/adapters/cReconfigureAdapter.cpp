 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cReconfigureAdapter.cpp
 *
 *  Created on: June 16, 2018
 *      Author: Erik Kouters
 */

#include "ext/ballHandlingNames.hpp"
#include "int/adapters/cReconfigureAdapter.hpp"
#include "int/types/ballHandlingSettingsType.hpp"
#include "tracing.hpp"

cReconfigureAdapter::cReconfigureAdapter(ballHandlingControl* bhControl)
{
    TRACE(">");
    _bhControl = bhControl;
    TRACE("<");
}

cReconfigureAdapter::~cReconfigureAdapter()
{
}

void cReconfigureAdapter::initializeRC()
{
    TRACE(">");

    /* Create reconfigure server */
    _srv.reset(new dynamic_reconfigure::Server<ballHandling::ballHandlingConfig>());

    /* Bind the reconfiguration function */
    _f = boost::bind(&cReconfigureAdapter::reconfig_cb, this, _1, _2);
    _srv->setCallback(_f);

    TRACE("<");
}

void cReconfigureAdapter::reloadParams()
{
    // Create the NodeHandle to read the parameters from
    ros::NodeHandle nh = ros::NodeHandle(ballHandlingNames::nodename);

    // Read parameters from NodeHandle
    ballHandling::ballHandlingConfig config;
    config.__fromServer__(nh);

    // Trigger dynamic_reconfigure::Server and reconfig_cb such that the values loaded by YAML are written to _ppData.
    _srv->updateConfig(config);
    reconfig_cb(config, 0);
}

void cReconfigureAdapter::reconfig_cb(ballHandling::ballHandlingConfig &config, uint32_t level)
{
    TRACE(">");

    ballHandlingSettingsType newSettings;
    newSettings.armLiftedAngleThreshold = config.ArmLiftedAngleThreshold;
    newSettings.angleSetpoint = config.AngleSetpoint;
    newSettings.enableExtraPullForceWhenSingleArmUp = config.EnableExtraPullForceWhenSingleArmUp;
    newSettings.enableRobotVelocityFeedForward = config.EnableRobotVelocityFeedForward;
    newSettings.xFeedForwardVelocityFactor = config.XFeedForwardVelocityFactor;
    newSettings.yFeedForwardVelocityFactor = config.YFeedForwardVelocityFactor;
    newSettings.thetaFeedForwardVelocityFactor = config.ThetaFeedForwardVelocityFactor;
    newSettings.extraPullForceVelocity = config.ExtraPullForceVelocity;
    newSettings.disableExtraPullForceWhenBallDetected = config.DisableExtraPullForceWhenBallPossession;
    newSettings.disableRobotVelocityFeedForwardWhenArmsNotUp = config.DisableRobotVelocityFeedForwardWhenNoBallPossession;
    newSettings.leftArmCalibrationUp = config.LeftArmCalibrationUp;
    newSettings.leftArmCalibrationDown = config.LeftArmCalibrationDown;
    newSettings.rightArmCalibrationUp = config.RightArmCalibrationUp;
    newSettings.rightArmCalibrationDown = config.RightArmCalibrationDown;
    newSettings.ballPossessionAngleThresholdOff = config.BallPossessionAngleThresholdOff;
    newSettings.ballPossessionAngleThresholdOn = config.BallPossessionAngleThresholdOn;
    newSettings.ballPossessionTimeInterval = config.BallPossessionTimeInterval;

    _bhControl->set_settings(newSettings);

    TRACE("<");
}
