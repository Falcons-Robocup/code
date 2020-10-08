 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanning.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */
#include <boost/bind.hpp>

// own package
#include "int/VelocitySetpointControllers.hpp"
#include "int/PathPlanning.hpp"
#include "int/PathPlanningAlgorithms.hpp"

// other Falcons packages
#include "tracing.hpp"
#include "cDiagnostics.hpp"


PathPlanning::PathPlanning(CFI *configInterface, InputInterface *inputInterface, OutputInterface *outputInterface)
{
    TRACE_FUNCTION("");
    _configInterface = configInterface;
    _inputInterface = inputInterface;
    _outputInterface = outputInterface;
    if (_configInterface != NULL)
    {
        _configInterface->get(data.config);
    }
}

PathPlanning::~PathPlanning()
{
}

actionResultTypeEnum PathPlanning::iterate()
{
    TRACE_FUNCTION("");

    // clear intermediate and output data, retrieve configuration
    prepare();

    // get inputs from input interface
    getInputs();

    // configure motion profile limits based on slow flag and on ball possession
    data.configureLimits();

    // calculate
    data.timestamp = ftime::now();
    auto result = calculate();

    // write outputs to output interface
    setOutputs();

    // wrap up
    return result;
}

actionResultTypeEnum PathPlanning::calculate()
{
    // this function assumes all inputs are set (see iterate() wrapper)

    TRACE_FUNCTION("");

    data.traceInputs();

    // when was the last time PathPlanning was poked?
    double age = data.timestamp - data.previousTimestamp;
    if (age > 2*data.dt)
    {
        // too long ago, probably robot was standing still for a while
        // (in which case execution architecture causes components to not be poked)
        // -> reset velocity in order to properly limit acceleration
        data.previousVelocityRcs = Velocity2D(0.0, 0.0, 0.0);
    }

    // configure the sequence of algorithms
    std::vector<PathPlanningAlgorithm *> algorithms; // previously known as 'blocks'

    // basics
    algorithms.push_back(new RequireWorldModelActive());
    algorithms.push_back(new CheckStopCommand());
    algorithms.push_back(new CheckTargetValid());
    algorithms.push_back(new CheckTargetReached());

    // subtarget calculation
    algorithms.push_back(new EscapeForbiddenAreas());
    algorithms.push_back(new CalculateObstacles()); // opponents + teammembers + forbiddenAreas + projectedSpeedVectors
    algorithms.push_back(new AvoidObstacles());
    //algorithms.push_back(new Shielding());
    algorithms.push_back(new ForwardDriving());

    // prepare for velocity control to sub-target
    algorithms.push_back(new ShiftBallOffset());
    algorithms.push_back(new CalculateDeltas());

    // velocity control (in FCS or RCS, depends on configuration)
    // roadmap: this part is probably going to be moved to lower-level motion control
    // new motion platform might implement this using a setpoint generator on lower level (?),
    // but for simulation we will always need this, so then it would have to become switchable
    algorithms.push_back(new SelectVelocityController());
    algorithms.push_back(new CalculateVelocity(boost::bind(&PathPlanning::setupAndGetVelocitySetpointController, this))); // just-in-time callback
    // limiters and everything else below is performed in RCS
    algorithms.push_back(new CalculateAccelerating());
    algorithms.push_back(new ApplyLimits());
    algorithms.push_back(new UnShiftBallOffset());
    algorithms.push_back(new CalculateDeltas()); // needed for deadzone in case ball shift was applied
    algorithms.push_back(new Deadzone());

    // initially, the path consists of just the given target
    data.path.push_back(data.target);

    // execute the sequence of algorithms
    for (auto it = algorithms.begin(); it != algorithms.end(); ++it)
    {
        if (!data.done)
        {
            (*it)->execute(data);
        }
    }

    data.traceOutputs();
    WRITE_TRACE;

    return data.resultStatus;
}

void PathPlanning::getInputs()
{
    TRACE_FUNCTION("");
    // configuration is handled at construction and upon change
    if (_inputInterface != NULL)
    {
        // query RTDB once so we could do repeated gets
        _inputInterface->fetch();
        // get and store data
        motionSetpoint sp = _inputInterface->getMotionSetpoint();
        data.target.pos = pose();
        data.stop = true;
        if (sp.action == actionTypeEnum::MOVE) // for any other action: do nothing
        {
            data.target.pos.x = sp.position.x;
            data.target.pos.y = sp.position.y;
            data.target.pos.Rz = sp.position.z;
            data.slow = sp.slow; // slow and setpiece are synonym
            data.stop = false;
        }
        data.target.vel = pose(); // nonzero input velocity is not yet supported on external interface
        data.forbiddenAreas = _inputInterface->getForbiddenAreas();
        data.addForbiddenAreas(data.forbiddenAreas); // add to calculatedForbiddenAreas
        data.robot = _inputInterface->getRobotState();
        data.teamMembers = _inputInterface->getTeamMembers();
        data.obstacles = _inputInterface->getObstacles();
        data.balls = _inputInterface->getBalls();
    }
}

void PathPlanning::setOutputs()
{
    TRACE_FUNCTION("");
    if (_outputInterface != NULL)
    {
        robotVelocity sp;
        sp.x = data.resultVelocityRcs.x;
        sp.y = data.resultVelocityRcs.y;
        sp.Rz = data.resultVelocityRcs.phi;
        _outputInterface->setVelocity(data.resultStatus, sp);
        _outputInterface->setDiagnostics(makeDiagnostics());
    }
}

diagPathPlanning PathPlanning::makeDiagnostics()
{
    // data adapter
    diagPathPlanning result;
    result.numCalculatedObstacles = data.calculatedObstacles.size();
    result.path = data.path;
    result.forbiddenAreas = data.calculatedForbiddenAreas;
    result.distanceToSubTargetRCS.x = data.deltaPositionRcs.x;
    result.distanceToSubTargetRCS.y = data.deltaPositionRcs.y;
    result.distanceToSubTargetRCS.Rz = data.deltaPositionRcs.phi;
    result.accelerationRCS = data.accelerationRcs;
    result.accelerationClipping.resize(3);
    result.isAccelerating.resize(3);
    result.deadzone.resize(3);
    for (int dof = 0; dof < 3; ++dof)
    {
        result.isAccelerating[dof] = data.isAccelerating[dof];
        result.accelerationClipping[dof] = data.accelerationClipping[dof];
        result.deadzone[dof] = data.deadzone[dof];
    }
    result.shortStroke = data.shortStroke;
    result.pid = data.pidState;
    return result;
}

void PathPlanning::prepare()
{
    TRACE_FUNCTION("");
    data.reset();

    // new configuration?
    if (_configInterface != NULL)
    {
        bool config_changed = _configInterface->get(data.config);
        if (config_changed)
        {
            // cleanup
            clearVelocitySetpointController();
        }
    }
    data.vcConfig.type = VelocitySetpointControllerTypeEnum::NONE;
    // configure motion profile limits based on configuration
    data.configureLimits();
    // timestepping
    if (data.config.nominalFrequency > 0)
    {
        data.dt = 1.0 / data.config.nominalFrequency;
    }
    else
    {
        data.dt = 1.0 / 20;
    }
    TRACE("nominalFrequency=%.1f dt=%.4fs", data.config.nominalFrequency, data.dt);
}

void PathPlanning::clearVelocitySetpointController()
{
    TRACE_FUNCTION("");
    if (_velocitySetpointController != NULL)
    {
        delete _velocitySetpointController;
    }
    _velocitySetpointController = NULL;
    _currentVelocitySetpointControllerType = VelocitySetpointControllerTypeEnum::NONE;
}

void PathPlanning::setupVelocitySetpointController()
{
    TRACE_FUNCTION("");
    bool needReInit = (_velocitySetpointController == NULL) ||
        (data.vcConfig.type != _currentVelocitySetpointControllerType);
    // perform the re-init?
    if (needReInit)
    {
        clearVelocitySetpointController();
        TRACE("switching to velocitySetpointControllerType %s", enum2str(data.vcConfig.type));
        switch (data.vcConfig.type)
        {
            case VelocitySetpointControllerTypeEnum::NONE:
                _velocitySetpointController = new StopVelocitySetpointController();
                break;
            case VelocitySetpointControllerTypeEnum::LINEAR:
                _velocitySetpointController = new LinearVelocitySetpointController();
                break;
            case VelocitySetpointControllerTypeEnum::PID:
                _velocitySetpointController = new PIDVelocitySetpointController(data.config.pid);
                break;
            case VelocitySetpointControllerTypeEnum::SPG:
                if (data.vcConfig.coordinateSystem == CoordinateSystemEnum::FCS)
                {
                    TRACE_ERROR("combination of configuration parameters is not supported: SPG and FCS");
                }
                TRACE("SPG config = %s", tostr(data.config.setPointGenerator).c_str());
                _velocitySetpointController = new SPGVelocitySetpointController(data.config.setPointGenerator);
                break;
            default:
                TRACE_ERROR("unknown configuration enum value (%d)", (int)data.vcConfig.type);
        }
        _currentVelocitySetpointControllerType = data.vcConfig.type;
    }
}

AbstractVelocitySetpointController *PathPlanning::getVelocitySetpointController()
{
    TRACE_FUNCTION("");
    return _velocitySetpointController;
}

AbstractVelocitySetpointController *PathPlanning::setupAndGetVelocitySetpointController()
{
    // this function should be called just-in-time, from CalculateVelocity
    TRACE_FUNCTION("");
    setupVelocitySetpointController();
    TRACE("using velocitySetpointControllerType %s", enum2str(_currentVelocitySetpointControllerType));
    return getVelocitySetpointController();
}


