// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * velocityControlTestDefaults.cpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */


// Include testframework
#include "velocityControlTestDefaults.hpp"
#include "ConfigRTDBAdapter.hpp" // configuration
#include "tracing.hpp"


// common setup and some config values (we don't want to be sensitive to production yamls)

class ConfigStubPP : public ppCFI
{
public:
    bool get(ConfigPathPlanning &ppConfig);
};

bool ConfigStubPP::get(ConfigPathPlanning &ppConfig)
{
    TRACE_FUNCTION("");

    ppConfig.forwardDriving.withoutBall.enabled = false;
    ppConfig.forwardDriving.withoutBall.minimumDistance = 3.0;
    ppConfig.forwardDriving.withBall.enabled = true;
    ppConfig.forwardDriving.withBall.minimumDistance = 2.0;
    ppConfig.deadzone.toleranceXY = 0.05;
    ppConfig.deadzone.toleranceRz = 0.01;

    return true;
}

class ConfigStubVC : public vcCFI
{
public:
    bool get(ConfigVelocityControl &vcConfig);
};

bool ConfigStubVC::get(ConfigVelocityControl &vcConfig)
{
    TRACE_FUNCTION("");

    vcConfig.nominalFrequency = 20;

    ///// NORMAL

    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].velocityControllers.threshold = 5.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].velocityControllers.longStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].velocityControllers.longStroke.coordinateSystem = CoordinateSystemEnum::FCS;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::SPG;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].velocityControllers.shortStroke.coordinateSystem = CoordinateSystemEnum::RCS;

    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.XY_P = 3.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.XY_I = 0.01;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.XY_D = 0.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.RZ_P = 6.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.RZ_I = 5.5;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.RZ_D = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.maxI_XY = 0.5;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.maxI_Rz = 0.1;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.fadeI_XY = 1.00;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].pid.fadeI_Rz = 0.98;

    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxDecX = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxDecY = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxDecRz = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.accThresholdX = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.accThresholdY = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.accThresholdRz = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxVelX = 2.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxVelYforward = 2.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxVelYbackward = 2.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxVelRz = 2.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxAccX = 200.0; // all accelerations: practically infinite, convenient for most tests
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxAccYforward = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxAccYbackward = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::NORMAL)].limits.maxAccRz = 200.0;

    ////// WITH_BALL

    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].velocityControllers.threshold = 5.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].velocityControllers.longStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].velocityControllers.longStroke.coordinateSystem = CoordinateSystemEnum::FCS;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::SPG;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].velocityControllers.shortStroke.coordinateSystem = CoordinateSystemEnum::RCS;

    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.XY_P = 3.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.XY_I = 0.01;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.XY_D = 0.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.RZ_P = 6.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.RZ_I = 5.5;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.RZ_D = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.maxI_XY = 0.5;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.maxI_Rz = 0.1;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.fadeI_XY = 1.00;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].pid.fadeI_Rz = 0.98;

    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxDecX = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxDecY = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxDecRz = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.accThresholdX = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.accThresholdY = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.accThresholdRz = 0.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxVelX = 1.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxVelYforward = 1.7;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxVelYbackward = 1.3;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxVelRz = 1.1;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxAccX = 200.0; // all accelerations: practically infinite, convenient for most tests
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxAccYforward = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxAccYbackward = 200.0;
    vcConfig.motionTypes[enum2str(motionTypeEnum::WITH_BALL)].limits.maxAccRz = 200.0;

    return true;
}

VelocityControl velocityControlSetup(vcCFI *vcConfig, ppCFI *ppConfig, OutputInterface *output)
{
    TRACE_FUNCTION("");
    auto pp = VelocityControl(vcConfig, ppConfig, NULL, output);
    pp.prepare();
    pp.data.robot.status = robotStatusEnum::INPLAY;
    pp.data.robot.hasBall = false;
    pp.data.robot.position = pose(0, 0, 0);
    pp.data.robot.velocity = pose(0, 0, 0);
    pp.data.target.pos = pose(0, 0, 0);
    pp.data.target.vel = pose(0, 0, 0);
    pp.data.robotPosVelMoveType = robotPosVelEnum::POSVEL;
    pp.data.motionType = motionTypeEnum::NORMAL;
    pp.data.configureLimits();
    return pp;
}

VelocityControl defaultVelocityControlSetup()
{
    TRACE_FUNCTION("");
    ConfigStubPP configStubPP;
    ConfigStubVC configStubVC;
    return velocityControlSetup(&configStubVC, &configStubPP);
}
