 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * pathPlanningTestDefaults.cpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */


// Include testframework
#include "pathPlanningTestDefaults.hpp"
#include "ConfigRTDBAdapter.hpp" // configuration
#include "tracing.hpp"


// common setup and some config values (we don't want to be sensitive to production yamls)

class ConfigStub : public CFI
{
public:
    bool get(ConfigPathPlanning &c);
};

bool ConfigStub::get(ConfigPathPlanning &c)
{
    TRACE_FUNCTION("");
    c.nominalFrequency = 20;
    c.velocityControllers.threshold = 5.0;
    c.velocityControllers.longStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    c.velocityControllers.longStroke.coordinateSystem = CoordinateSystemEnum::FCS;
    c.velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::SPG;
    c.velocityControllers.shortStroke.coordinateSystem = CoordinateSystemEnum::RCS;
    c.obstacleAvoidance.enabled = true;
    c.obstacleAvoidance.distanceScalingFactor = 0.0;
    c.obstacleAvoidance.speedScalingFactor = 1.0;
    c.obstacleAvoidance.speedLowerThreshold = 0.2;
    c.obstacleAvoidance.speedUpperThreshold = 4.0;
    c.obstacleAvoidance.ballClearance = 1.0;
    c.obstacleAvoidance.generatedObstacleSpacing = 0.5;
    c.obstacleAvoidance.groupGapDistance = 0.622;
    c.obstacleAvoidance.subTargetDistance = 0.571;
    c.obstacleAvoidance.subTargetExtensionFactor = 0.0;
    c.slowFactor = 0.5;
    c.forwardDriving.withoutBall.enabled = false;
    c.forwardDriving.withoutBall.minimumDistance = 3.0;
    c.forwardDriving.withBall.enabled = true;
    c.forwardDriving.withBall.minimumDistance = 2.0;
    c.limits.enabled = true;
    c.limits.withoutBall.maxVelX = 2.0;
    c.limits.withoutBall.maxVelYforward = 2.0;
    c.limits.withoutBall.maxVelYbackward = 2.0;
    c.limits.withoutBall.maxVelRz = 2.0;
    c.limits.withoutBall.maxAccX = 200.0; // all accelerations: practically infinite, convenient for most tests
    c.limits.withoutBall.maxAccYforward = 200.0;
    c.limits.withoutBall.maxAccYbackward = 200.0;
    c.limits.withoutBall.maxAccRz = 200.0;
    c.limits.withBall.maxVelX = 1.3;
    c.limits.withBall.maxVelYforward = 1.7;
    c.limits.withBall.maxVelYbackward = 1.3;
    c.limits.withBall.maxVelRz = 1.1;
    c.limits.withBall.maxAccX = 200.0; // all accelerations: practically infinite, convenient for most tests
    c.limits.withBall.maxAccYforward = 200.0;
    c.limits.withBall.maxAccYbackward = 200.0;
    c.limits.withBall.maxAccRz = 200.0;
    c.limits.common.maxDecX = 200.0;
    c.limits.common.maxDecY = 200.0;
    c.limits.common.maxDecRz = 200.0;
    c.limits.common.toleranceXY = 0.05;
    c.limits.common.toleranceRz = 0.01;
    c.limits.common.accThresholdX = 0.3;
    c.limits.common.accThresholdY = 0.3;
    c.limits.common.accThresholdRz = 0.3;

    // NOTE: skipping setpiece configuration, not much added test coverage due to robust switching in code
    c.boundaries.targetInsideForbiddenArea = BoundaryOptionEnum::STOP_AND_FAIL;
    c.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    c.boundaries.fieldMarginX = 0.0;
    c.boundaries.fieldMarginY = 0.0;
    c.boundaries.targetOnOwnHalf = BoundaryOptionEnum::ALLOW;
    c.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::ALLOW;
    return true;
}

PathPlanning pathPlanningSetup(CFI *ci, OutputInterface *output)
{
    TRACE_FUNCTION("");
    auto pp = PathPlanning(ci, NULL, output);
    pp.prepare();
    pp.data.robot.status = robotStatusEnum::INPLAY;
    pp.data.robot.hasBall = false;
    pp.data.robot.position = pose(0, 0, 0);
    pp.data.robot.velocity = pose(0, 0, 0);
    pp.data.target.pos = pose(0, 0, 0);
    pp.data.target.vel = pose(0, 0, 0);
    pp.data.configureLimits();
    return pp;
}

PathPlanning defaultPathPlanningSetup()
{
    TRACE_FUNCTION("");
    ConfigStub configStub;
    return pathPlanningSetup(&configStub);
}

PathPlanning yamlPathPlanningSetup(std::string const &yamlfilename, OutputInterface *output)
{
    TRACE_FUNCTION("");
    ConfigRTDBAdapter<ConfigPathPlanning> cadp(CONFIG_PATHPLANNING, true); // test mode: no tprintf and no wait_for_put thread
    cadp.loadYAML(pathToConfig() + "/" + yamlfilename);
    return pathPlanningSetup(&cadp, output);
}

PathPlanning ConfigPathPlanningSetup(ConfigPathPlanning const &config, OutputInterface *output)
{
    TRACE_FUNCTION("");
    ConfigInterface<ConfigPathPlanning> cfi;
    cfi.set(config);
    return pathPlanningSetup(&cfi, output);
}

ConfigPathPlanning loadYAML(std::string const &yamlfilename)
{
    TRACE_FUNCTION("");
    // TODO: use some json/yaml library to directly load ... or we write something ourselves ...
    // workaround: use ConfigAdapter, which underwater uses loadYAML.py ...
    ConfigRTDBAdapter<ConfigPathPlanning> cadp(CONFIG_PATHPLANNING, true); // test mode: no tprintf and no wait_for_put thread
    std::string f = pathToConfig() + "/" + yamlfilename;
    TRACE("loading yaml from %s", f.c_str());
    cadp.loadYAML(f);
    ConfigPathPlanning result;
    (void)cadp.get(result);
    return result;
}

ConfigPathPlanning adaptConfigToSim(ConfigPathPlanning const &config)
{
    TRACE_FUNCTION("");
    auto result = config;
    result.setPointGenerator.convergenceWorkaround = true;
    result.setPointGenerator.latencyOffset = 0.0;
    result.deadzone = false; // workaround for minisimulation 18 not converging for some reason... seems linked with SPG internal tolerances?
    return result;
}

