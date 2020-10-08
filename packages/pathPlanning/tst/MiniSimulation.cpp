 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MiniSimulation.cpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */


#include "int/adapters/RTDBOutputAdapter.hpp"
#include "MiniSimulation.hpp"
#include "pathPlanningTestDefaults.hpp"
#include <boost/filesystem.hpp>
#include "FalconsRtDB2.hpp"
#include "tracing.hpp"
#include "ftime.hpp"


MiniSimulation::MiniSimulation(std::string const &yamlfile, float timeout, bool writeRDL)
{
    TRACE_FUNCTION("");
    _config = loadYAML(yamlfile);
    TRACE("_config=%s", tostr(_config).c_str());
    _timeout = timeout;
    _writeRDL = writeRDL;
}

MiniSimulation::MiniSimulation(ConfigPathPlanning const &config, float timeout, bool writeRDL)
{
    TRACE_FUNCTION("");
    _config = config;
    TRACE("_config=%s", tostr(_config).c_str());
    _timeout = timeout;
    _writeRDL = writeRDL;
}

void MiniSimulation::initialize()
{
    TRACE_FUNCTION("");
    if (_initialized)
    {
        return;
    }
    if (_writeRDL)
    {
        // clear RTDB for clean logging, must be done before setting up PP for loadYAML
        //TODO: broken? systemStdout("rtdbClear");
        // setup RTDB adapters
        for (int robotId = 0; robotId <= 5; robotId++)
        {
            _rtdb[robotId] = RtDB2Store::getInstance().getRtDB2(robotId, 'A');
        }
        _rtdbOutputAdapter = new RTDBOutputAdapter(false, _myRobotId);
    }
    _pp = ConfigPathPlanningSetup(_config, _rtdbOutputAdapter);
    _initialized = true;
}

void MiniSimulation::setScene(PathPlanningSimulationScene &scene)
{
    TRACE_FUNCTION("");
    _scene = scene;
    _myRobotId = scene.robotId;
}

void MiniSimulation::setScene(std::string const &scenefile)
{
    TRACE_FUNCTION("");
    auto scene = loadScene(scenefile);
    setScene(scene);
}

PathPlanningSimulationScene MiniSimulation::loadScene(std::string const &scenefile)
{
    TRACE_FUNCTION("");
    // guess location of the file
    std::string f = scenefile;
    if (!boost::filesystem::exists(f))
    {
        // TODO Turtleroot etc
        f = pathToCodeRepo() + "/packages/pathPlanning/tst/" + f;
    }
    if (!boost::filesystem::exists(f))
    {
        throw std::runtime_error("could not resolve file location");
    }
    throw std::runtime_error("not implemented"); // TODO. There is no yaml-to-rtdb-object converter yet. I (Jan) played with yaml-to-json but I could not get yaml-cpp to emit valid json... Best workaround we currently have is via python (see loadYAML and some ConfigAdapters)
    //return loadYAML<PathPlanningSimulationScene>(f);
    return PathPlanningSimulationScene(); // dummy
}

MiniSimulationResult MiniSimulation::run()
{
    TRACE_FUNCTION("");
    // initialize
    initialize();
    applySceneToData();
    _simulatedPositionFcs = Position2D(_pp.data.robot.position.x, _pp.data.robot.position.y, _pp.data.robot.position.Rz);
    _simulatedVelocityFcs = Velocity2D(_pp.data.robot.velocity.x, _pp.data.robot.velocity.y, _pp.data.robot.velocity.Rz);
    initRDL();
    _result.duration = 0.0;
    _result.distance = 0.0;
    _result.maxSpeedRobot = 0.0;
    _result.minDistanceToObstacles = 99.0;
    _result.maxVelBall.x = 0.0;
    _result.maxVelBall.y = 0.0;
    _result.maxAccBall.x = 0.0;
    _result.maxAccBall.y = 0.0;
    // iterate
    rtime t0 = ftime::now();
    rtime t = t0;
    float dt = _pp.data.dt;
    actionResultTypeEnum status = actionResultTypeEnum::RUNNING;
    bool success = true;
    int iteration = 0;
    int maxIteration = int(_timeout / dt);
    std::set<VelocitySetpointControllerTypeEnum> vcUsed;
    while (true)
    {
        // call pathPlanning
        _pp.data.reset(); // clear intermediate results
        // configure motion profile limits based on slow flag and on ball possession
        _pp.data.configureLimits(); // TODO: hide all of this in pp
        status = _pp.calculate();
        vcUsed.insert(_pp.data.vcConfig.type);
        tickRDL(t);
        // update simulated velocity and position
        processIterationResult();
        // done?
        if (status != actionResultTypeEnum::RUNNING)
        {
            break;
        }
        // advance time for next iteration
        iteration++;
        t = t0 + iteration * dt;
        _pp.data.timestamp += dt;
        // timeout?
        if (iteration > maxIteration)
        {
            status = actionResultTypeEnum::FAILED;
            success = false;
            break;
        }
    }
    finishRDL();
    // store results
    _result.status = status;
    _result.success = success;
    _result.duration = double(t - t0);
    // write to stdout
    std::string vcStr = enum2str(*vcUsed.begin());
    if (vcUsed.size() > 1) vcStr += "+";
    tprintf("mini simulation finished: success=%d status=%s duration=%.2fs distance=%.2fm vc=%s", _result.success, enum2str(_result.status), _result.duration, _result.distance, vcStr.c_str());
    //tprintf("   misc: maxSpeedRobot=%.2fm/s minDistanceToObstacles=%.2fm maxVelBall=(%.2f, %.2f)m/s maxAccBall=(%.2f, %.2f)m/s2 ", _result.maxSpeedRobot, _result.minDistanceToObstacles, _result.maxVelBall.x, _result.maxVelBall.y, _result.maxAccBall.x, _result.maxAccBall.y);
    return _result;
}

robotState makeRobotState(SimulationSceneRobot robot)
{
    robotState result;
    result.status = robotStatusEnum::INPLAY;
    result.timestamp = rtime(0);
    result.position = robot.position;
    result.velocity = robot.velocity;
    result.robotId = robot.robotId;
    result.hasBall = false;
    result.ballAcquired = vec2d(0.0, 0.0);
    result.teamId = "A";
    return result;
}

obstacleResult makeObstacleResult(SimulationSceneRobot robot)
{
    TRACE_FUNCTION("");
    obstacleResult result;
    result.position = vec2d(robot.position.x, robot.position.y);
    result.velocity = vec2d(robot.velocity.x, robot.velocity.y);
    result.confidence = 1.0;
    result.id = 0;
    return result;
}

obstacleResult makeObstacleResult(SimulationSceneObstacle obstacle)
{
    obstacleResult result;
    result.position = vec2d(obstacle.position.x, obstacle.position.y);
    result.velocity = vec2d(obstacle.velocity.x, obstacle.velocity.y);
    result.confidence = 1.0;
    result.id = 0;
    return result;
}

void MiniSimulation::applySceneToData()
{
    _pp.data.reset();
    int robotId = _scene.robotId;
    bool hasBall = _scene.hasBall;
    _pp.data.target = _scene.target;
    _pp.data.forbiddenAreas = _scene.forbiddenAreas;
    if (_scene.balls.size() > 0)
    {
        ballResult b;
        b.position = _scene.balls.at(0).position;
        b.velocity = _scene.balls.at(0).velocity;
        b.confidence = 1.0;
        b.owner.type = (hasBall ? ballPossessionTypeEnum::TEAM : ballPossessionTypeEnum::FIELD);
        b.owner.robotId = (hasBall ? robotId : -1);
        _pp.data.balls.push_back(b);
    }
    for (auto& robot: _scene.robots)
    {
        robotState r = makeRobotState(robot);
        if (r.robotId == robotId)
        {
            r.hasBall = hasBall;
            _pp.data.robot = r;
        }
        else
        {
            _pp.data.teamMembers.push_back(r);
        }
    }
    for (auto& robot: _scene.opponents)
    {
        _pp.data.obstacles.push_back(makeObstacleResult(robot));
    }
    for (auto& obst: _scene.obstacles)
    {
        _pp.data.obstacles.push_back(makeObstacleResult(obst));
    }
}

void MiniSimulation::overruleVelocityController(VelocitySetpointControllerTypeEnum const &vct, CoordinateSystemEnum const &vccs)
{
    _pp.data.config.velocityControllers.shortStroke.type = vct;
    _pp.data.config.velocityControllers.shortStroke.coordinateSystem = vccs;
    _pp.data.config.velocityControllers.threshold = 99;
    _pp.data.vcConfig.type = vct;
    _pp.data.vcConfig.coordinateSystem = vccs;
}

void imin(float &vmin, float v)
{
    if (v < vmin) vmin = v;
}
void imax(float &vmax, float v)
{
    if (v > vmax) vmax = v;
}

float MiniSimulation::calcDistanceToObstacles()
{
    float result = 99;
    Vector2D r(_simulatedPositionFcs.x, _simulatedPositionFcs.y);
    for (auto& obst: _scene.obstacles)
    {
        Vector2D o(obst.position.x, obst.position.y);
        imin(result, (r - o).size());
    }
    return result;
}

void MiniSimulation::processIterationResult()
{
    // update simulation
    _simulatedVelocityFcs = _pp.data.resultVelocityRcs;
    _simulatedVelocityFcs.transform_rcs2fcs(_simulatedPositionFcs);
    _simulatedPositionFcs.update(_simulatedVelocityFcs, _pp.data.dt);
    // update robot administration
    _pp.data.robot.position = pose(_simulatedPositionFcs.x, _simulatedPositionFcs.y, _simulatedPositionFcs.phi);
    _pp.data.robot.velocity = pose(_simulatedVelocityFcs.x, _simulatedVelocityFcs.y, _simulatedVelocityFcs.phi);
    // add noise?
    _pp.data.robot.position.x += (2.0 * rand() / RAND_MAX - 1.0) * _scene.localizationNoise.xy;
    _pp.data.robot.position.y += (2.0 * rand() / RAND_MAX - 1.0) * _scene.localizationNoise.xy;
    _pp.data.robot.position.Rz += (2.0 * rand() / RAND_MAX - 1.0) * _scene.localizationNoise.Rz;
    // calculate distance of this iteration for total travelled path calculation
    float speedRobot = _simulatedVelocityFcs.xy().size();
    _result.distance += speedRobot * _pp.data.dt;
    imax(_result.maxSpeedRobot, speedRobot);
    // calculate other statistics, useful for evaluating the simulation result
    imin(_result.minDistanceToObstacles, calcDistanceToObstacles());
    // verify: calculate velocity and acceleration of the ball in RCS
    if (_pp.data.robot.hasBall)
    {
        Velocity2D ballVelocity = _pp.data.resultVelocityRcs;
        ballVelocity.x += ROBOT_RADIUS * -_pp.data.resultVelocityRcs.phi;
        imax(_result.maxVelBall.x, fabs(ballVelocity.x));
        imax(_result.maxVelBall.y, fabs(ballVelocity.y));
        /* TODO: robust checks on ball acceleration -- require a bit more algorithm robustness, we get small occasional spikes / discontinuities
        static Velocity2D ballVelocityPrev;
        static bool havePrev = false;
        if (havePrev)
        {
            auto a = (ballVelocity - ballVelocityPrev) / _pp.data.dt;
            // only consider if PathPlanning was accelerating, not braking (separate limiters)
            if (_pp.data.isAccelerating[0] && _pp.data.isAccelerating[1] && _pp.data.isAccelerating[2])
            {
                imax(_result.maxAccBall.x, fabs(a.x));
                imax(_result.maxAccBall.y, fabs(a.y));
                //tprintf("rvel=(%6.2f,%6.2f,%6.2f) bvel=(%6.2f,%6.2f) bacc=(%6.2f,%6.2f)", _pp.data.resultVelocityRcs.x, _pp.data.resultVelocityRcs.y, _pp.data.resultVelocityRcs.phi, ballVelocity.x, ballVelocity.y, a.x, a.y);
            }
        }
        ballVelocityPrev = ballVelocity;
        havePrev = true;
        */
    }
}

void MiniSimulation::initRDL()
{
    if (!_writeRDL)
    {
        return;
    }
    // setup the logger
    _logger = new cLogger();
    _logger->setFrequency(0); // not applicable, we will run as fast as possible
    if (boost::filesystem::exists(_rdlFilename))
    {
        boost::filesystem::remove(_rdlFilename);
        // otherwise file will be partly overwritten ... recipe for disaster ...
    }
    _logger->writeToFile(_rdlFilename);
}

void MiniSimulation::tickRDL(rtime const &t)
{
    if (!_writeRDL || _logger == NULL)
    {
        return;
    }
    // PathPlanning construction was done with an RTDBOutputAdapter, let's poke it
    _pp.setOutputs();
    // here we make sure also pathPlanning inputs are in RTDB (robots, obstacles and such)
    rtdbPut();
    // call logger to write the frame, based on complete RTDB contents
    _logger->tick(t);
}

void MiniSimulation::finishRDL()
{
    if (!_writeRDL || _logger == NULL)
    {
        return;
    }
    delete _logger;
}

void MiniSimulation::rtdbPut()
{
    // for visualization / diagnostics purposes
    _rtdb[_myRobotId]->put(ROBOT_STATE, &_pp.data.robot);
    for (auto robot: _pp.data.teamMembers)
    {
        _rtdb[robot.robotId]->put(ROBOT_STATE, &robot);
    }
    _rtdb[_myRobotId]->put(FORBIDDEN_AREAS, &_pp.data.forbiddenAreas);
    // 0 = visualization (coach)
    auto balls = _pp.data.balls; // might be omitted, check robot property
    if (_pp.data.robot.hasBall && balls.size() == 0)
    {
        // TODO: refactor worldModel such that we can reuse its calculation
        // see adaptersCollector::ballPossessionOverrule
    }
    _rtdb[0]->put(BALLS, &balls);
    _rtdb[0]->put(OBSTACLES, &_pp.data.obstacles);
}

