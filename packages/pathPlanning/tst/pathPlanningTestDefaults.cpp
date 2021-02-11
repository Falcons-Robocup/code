// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
