""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3
#
# Python library for commands, on-robot.
# This file implements the robot test interface connections.
#
# Implementations:
#    robotCLI.py          # command-line interface
#    robotLibrary.py      # discloses all basic commands and scenarios, also includes parser
#    robotScenarios.py    # scenario interface
#    robotInterface.py    # basic commands and their RTDB/ROS interfaces
#    scenarios/*.py       # scenario implementations
#
# Jan Feitsma, 2016-12-17
#
# TODO:
# 1. forcedStop: we explicitly have to call stop (zero speed setpoint) after most actions
#    perhaps it is better to implement a watchdog in simulator?
# 2. reconsider test execution architecture
#    currently, all commands will block main execution, and there is no way to interrupt a command (or a nested sequence)
#    it would be nice if it would be possible to do such, for instance:
#    * set a continuous speed setpoint (speed 0 0 1) until user presses ctrl-C
#    * set a target, but interrupt as soon as robot seems unable to get there
#    * run a scenario, interrupt if early bug is shown
#    this would probably require a more complex threading + signalhandling model ... who has ideas & energy to help?
# 3. improve rtdb interfacing, so we don't have to do things like rtdbGuessDb() -- linked to rtdb3
# 5. improve execution architecture: syncing motionPlanning set and getResult


# system includes
from time import sleep, time
import os
import threading

# other packages
import falconspy
from sharedTypes import * # generated enums
from worldState import WorldState
from FalconsCoordinates import Vec2d, Vec3d, RobotPose

# config
import math
from FalconsConfig import FalconsConfig

# RTDB
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH

# one object in this module
_robotInterface = None
_shutdown = False


# predicates for test execution architecture
# sometimes an action should never stop by itself (example: setSpeed without duration)
# sometimes after a timeout
# sometimes when certain position is reached (move)
# sometimes when actionResult is PASSED (or FAILED)
def stopNever(*args):
    return False

def stopAtOnce(*args):
    return True

class stopAfter():
    def __init__(self, duration):
        self.duration = duration
        self.start = time()
    def __call__(self, *args):
        elapsed = time() - self.start
        return elapsed > self.duration

class stopWhenNear():
    def __init__(self, x, y, phi, xyTol, phiTol):
        self.x = x
        self.y = y
        self.phi = phi
        self.xyTol = xyTol
        self.phiTol = phiTol
    def __call__(self, *args):
        return robotCloseBy(self.x, self.y, self.phi, self.xyTol, self.phiTol)


# entry point for clients
def connect(robotId):
    global _robotInterface
    _robotInterface = RobotInterface(robotId)
    global _shutdown
    _shutdown = False

def disconnect():
    shutDown()


class RobotInterface():
    def __init__(self, robotId):
        self.robotId = robotId
        assert(self.robotId in range(1, 20))

        # Simulated?
        hostname = os.uname()[1]
        self.isSimulated = bool("FALCON" or "TURTLE" in hostname)

        # RTDB
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False) # write mode
        self.rtdb2Store.refresh_rtdb_instances()
        self.rtdb_agent = self.robotId
        self.rtdb_key = None
        self.rtdb_value = None

        # World state
        self.worldState = WorldState(robotId)
        self.worldState.startMonitoring() # start its own thread

        # Execution architecture
        self.setMatchMode(False) # tell teamplay to not listen to heartbeat anymore, giving control to this library
        sleep(0.5) # avoid race condition: sleep for a while, to let current heartBeat finish ... otherwise immediate enableBallhandlers might not arrive...
        self.frequency = 30
        if self.isSimulated:
            self.frequency = 20 # needs to be consistent with the frequency set in package 'simulation' and pp limiters
        self.active = False
        self.stopCondition = stopNever

        # Static action counters, to make sure results match with setpoints
        self.mpActionId = 0
        self.tpControlId = 0

        # Configuration management
        rosNamespace = "/teamA/robot" + str(robotId) + "/"
        self.config = FalconsConfig(rosNamespace)

    def rtdbGet(self, key):
        return self.rtdb2Store.get(self.robotId, key)

    def rtdbPut(self, key, value):
        return self.rtdb2Store.put(self.robotId, key, value)

    def setMatchMode(self, matchMode):
        self.rtdbPut("MATCH_MODE", matchMode)

    def stimulate(self, rtdb_key, rtdb_value, stopCondition = stopNever):
        self.rtdb_key = rtdb_key
        self.rtdb_value = rtdb_value
        self.stopCondition = stopCondition
        self.active = True
        # blocking execution - see TODO(2)
        self.execute()

    def execute(self):
        dt = 1.0 / self.frequency
        while not isShutDown():
            if self.active:
                self.rtdbPut(self.rtdb_key, self.rtdb_value)
                if self.stopCondition(self.rtdb_key, self.rtdb_value):
                    self.active = False
            if not self.active:
                break
            sleep(dt) # TODO sleepUntil, avoid drift

def isSimulated():
    return _robotInterface.isSimulated

def isShutDown():
    return _shutdown

def shutDown():
    global _shutdown
    disableBallHandlers()
    _robotInterface.setMatchMode(True)
    _robotInterface.worldState.stopMonitoring()
    _shutdown = True

def stop():
    global _robotInterface
    _robotInterface.stimulate("ROBOT_VELOCITY_SETPOINT", [0.0, 0.0, 0.0], stopAtOnce)

def enableBallHandlers():
    global _robotInterface
    _robotInterface.stimulate("BALLHANDLERS_SETPOINT", True, stopAtOnce)

def disableBallHandlers():
    global _robotInterface
    _robotInterface.stimulate("BALLHANDLERS_SETPOINT", False, stopAtOnce)

def setVelocity(vx, vy, vphi, duration=0):
    """
    Send a velocity setpoint (RCS) to velocityControl.
    """
    global _robotInterface
    stopCondition = stopNever
    if duration > 0:
        stopCondition = stopAfter(duration)
    _robotInterface.stimulate("ROBOT_VELOCITY_SETPOINT", [vx, vy, vphi], stopCondition)
    # TODO(1) forcedStop
    # robot watchdog should make sure the robot stops, but it might have a small delay of say 0.1s
    # simulator does not have a watchdog (yet?)
    # so we have to call stop() directly, which sends a zero speed setpoint
    stop()

def mpActionFinished(actionResult, *args):
    return actionResult.result == actionResultTypeEnum.PASSED

class mpAction():
    def __init__(self, actionDef):
        global _robotInterface
        self.iteration = 0
        # sanitize types, rtdb serialization might be sensitive to it
        actionDef['action'] = int(actionDef['action'])
        actionDef['position'] = [float(actionDef['position'][0]), float(actionDef['position'][1]), float(actionDef['position'][2])]
        actionDef['id'] = _robotInterface.mpActionId
        actionDef['slow'] = bool(actionDef['slow'])
        actionDef['ballHandlersEnabled'] = bool(actionDef['ballHandlersEnabled'])
        self.status = actionResultTypeEnum.RUNNING
        _robotInterface.stimulate("ACTION", actionDef, self.evaluate)
        stop() # TODO(1) no watchdog in simulation

    def getResult(self, actionId):
        global _robotInterface
        item = _robotInterface.rtdbGet("ACTION_RESULT")
        actionResultId = item.value[0]
        actionResult = item.value[1]
        while actionResultId != actionId:
            # possible race condition ... we must wait until motionPlanning produced the result
            # this is a bit of a design flaw in current execution architecture -- TODO(5)
            # similar waiting happens within teamplay cRTDBInputAdapter::getActionResult
            sleep(0.001)
            item = _robotInterface.rtdbGet("ACTION_RESULT")
            actionResultId = item.value[0]
            actionResult = item.value[1]
        self.status = actionResult

    def evaluate(self, rtdb_key, rtdb_value):
        global _robotInterface
        # get mp action result
        self.getResult(rtdb_value["id"])
        # increment action id for next iteration
        self.iteration = 1 + self.iteration
        _robotInterface.mpActionId = 1 + _robotInterface.mpActionId
        rtdb_value["id"] = _robotInterface.mpActionId
        # continue if motionPlanning reports RUNNING, otherwise stop
        stop = (self.status != actionResultTypeEnum.RUNNING.value)
        return stop

def getBall(slow = False):
    mpAction({'action': actionTypeEnum.GET_BALL.value, 'position': [0, 0, 0], 'slow': slow, 'ballHandlersEnabled': True})

def mpMove(x, y, phi=0, xyTol=None, phiTol=None):
    """
    Basic move action via motionPlanning.
    """
    # TODO: do something with tolerances - reconfigure motionPlanning?
    mpAction({'action': actionTypeEnum.MOVE.value, 'position': [x, y, phi], 'slow': False, 'ballHandlersEnabled': True})

def move(x, y, phi=None, xyTol=0.1, phiTol=0.05):
    """
    Basic move action via pathPlanning. Uses worldModel to determine when done.
    """
    if phi == None:
        phi = ownPosition().Rz
    _robotInterface.stimulate("MOTION_SETPOINT", {'action': actionTypeEnum.MOVE.value, 'position': [x, y, phi], 'slow': False}, stopWhenNear(x, y, phi, xyTol, phiTol))

def passTo(x, y):
    """
    Pass via motionPlanning.
    """
    mpAction({'action': actionTypeEnum.PASS.value, 'position': [x, y, 0], 'slow': False, 'ballHandlersEnabled': True})

def passToTeamMember():
    teammembers = teamMembers();
    if len(teammembers) > 0:
        friendPos = getPosition(teammembers[-1]) # highest robot number rather than lowest (keeper)
        passTo(friendPos.x, friendPos.y)

def shootAt(x, y, z=0):
    """
    Straight shot via motionPlanning.
    """
    mpAction({'action': actionTypeEnum.SHOOT.value, 'position': [x, y, z], 'slow': False, 'ballHandlersEnabled': True})

def lobShotAt(x, y, z=0):
    """
    Lob shot via motionPlanning.
    """
    mpAction({'action': actionTypeEnum.LOB.value, 'position': [x, y, z], 'slow': False, 'ballHandlersEnabled': True})

def kick(power=30, height=0):
    """
    Kick via motionPlanning.
    """
    # abuse position argument tuple
    mpAction({'action': actionTypeEnum.KICK.value, 'position': [power, height, 0], 'slow': False, 'ballHandlersEnabled': True})

class tpOverride():
    # TODO: refactor common class out of mpAction and tpOverride
    def __init__(self, overrideDef):
        global _robotInterface
        self.iteration = 0
        overrideDef["id"] = _robotInterface.tpControlId
        self.status = behTreeReturnEnum.RUNNING
        # TODO: revise teamplay interfacing and execution architecture
        # ideally, we just do RTDB put on any level in teamplay (role, behavior, action) which would trigger appropriate code
        # but as a workaround we must set teamplay in standard heartbeat mode ..
        _robotInterface.setMatchMode(True)
        _robotInterface.stimulate("TP_OVERRIDE_STATE", overrideDef, self.evaluate)
        stop() # TODO(1) no watchdog in simulation
        _robotInterface.setMatchMode(False)

    def getResult(self, actionId):
        global _robotInterface
        item = _robotInterface.rtdbGet("TP_OVERRIDE_RESULT")
        if item != None:
            resultId = item.value[0]
            status = item.value[1]
        else:
            resultId = None
        while resultId != actionId:
            sleep(0.001)
            item = _robotInterface.rtdbGet("TP_OVERRIDE_RESULT")
            if item != None: # TODO refactor
                resultId = item.value[0]
                status = item.value[1]
            else:
                resultId = None
        self.status = status

    def evaluate(self, rtdb_key, rtdb_value):
        # get mp action result
        self.getResult(rtdb_value["id"])
        # increment action id for next iteration
        self.iteration = 1 + self.iteration
        _robotInterface.tpControlId = 1 + _robotInterface.tpControlId
        rtdb_value["id"] = _robotInterface.tpControlId
        # continue if RUNNING, otherwise stop
        stop = (self.status != behTreeReturnEnum.RUNNING.value)
        return stop

def behavior(behName):
    """
    Stimulate a behavior in teamplay.
    """
    # find tree enum value belonging to given name
    treeValue = treeEnum[behName].value
    params = {}
    tpOverride({'active': True, 'level': tpOverrideLevelEnum.BEHAVIOR.value, 'treeValue': treeValue, 'action': tpActionEnum.INVALID.value, 'params': params})

# joystick control
def joystickRelay():
    t = threading.Thread(target=joystickRelayThread)
    t.start()

def joystickRelayThread():
    dt = 1.0 / _robotInterface.frequency
    while not isShutDown():
        item = _robotInterface.rtdb2Store.get(0, "JOYSTICK_CONTROL")
        if item != None:
            if _robotInterface.robotId == item.value[0]: # command is intended for this robot
                _robotInterface.rtdbPut("ROBOT_VELOCITY_SETPOINT", item.value[1])
                _robotInterface.rtdbPut("BALLHANDLERS_SETPOINT", item.value[2])
                if item.value[4] > 0 and hasBall():
                    kick(item.value[4], item.value[3])
                action = item.value[5] # blocking actions
                if action == "getBall":
                    getBall()
                if action == "passToTeamMember":
                    passToTeamMember()
                if action == "shootAtGoal":
                    shootAt(0, 9, 0.7) # TODO use environmentField
        sleep(dt)
    

# worldState bindings

def activeRobots():
    L = _robotInterface.worldState.activeRobots()
    return L

def teamMembers():
    result = activeRobots()
    result.remove(myRobotId()) # inplace ...
    return result

def myRelIndex():
    return list(activeRobots()).index(_robotInterface.robotId)

def myRobotId():
    return int(_robotInterface.robotId)

def ballPossession():
    return _robotInterface.worldState.ballPossession()

def teamHasBall():
    bp = ballPossession()
    if bp == None:
        return False
    return bp == ballPossessionTypeEnum.TEAM

def opponentHasBall():
    bp = ballPossession()
    if bp == None:
        return False
    return bp == ballPossessionTypeEnum.OPPONENT

def hasBall(robotId = None):
    if robotId == None:
        robotId = myRobotId()
    return _robotInterface.worldState.hasBall(robotId)

def calculateTargetPhi(x, y):
    """
    Calculate robot orientation to face given position.
    """
    pos = getPosition()
    robotX = pos.x
    robotY = pos.y
    return math.atan2(y - robotY, x - robotX)

def getPosition(robotId = None):
    if robotId == None:
        robotId = myRobotId()
    pos = _robotInterface.worldState.getRobotPosition(robotId)
    return pos

def getVelocity(robotId = None):
    if robotId == None:
        robotId = myRobotId()
    vel = _robotInterface.worldState.getRobotVelocity(robotId)
    return vel

def ownPosition():
    return getPosition()

def ownVelocity():
    return getVelocity()

def ownSpeed():
    return getVelocity().xy().size()

def ballVelocity():
    return _robotInterface.worldState.getBallVelocity()

def ballPosition():
    return _robotInterface.worldState.getBallPosition()

def seeBall():
    """
    Does robot or team see the ball.
    """
    return (ballPosition() != None)

def ballOnSameHalf():
    ballpos = ballPosition()
    robotpos = ownPosition()
    if ballpos != None:
        return robotpos.y * ballpos.y > 0 # check if same sign
    return False

def robotCloseBy(x, y, phi=None, xyTol=0.1, phiTol=0.05):
    pos = ownPosition()
    xyDelta = (pos.xy() - Vec2d(x, y)).size()
    xyOk = xyDelta < xyTol
    phiOk = True
    # check phi?
    if phi != None:
        phiDelta = (pos - RobotPose(0, 0, phi)).Rz
        #print xyDelta, phiDelta
        phiOk = abs(phiDelta) < phiTol
    return xyOk and phiOk

def ballCloseBy(x, y, xyTol=0.1):
    bpos = ballPosition()
    if bpos == None:
        return False
    return (Vec2d(bpos.x, bpos.y) - Vec2d(x, y)).size() < xyTol

# special diagnostics

def bestVisionCandidate():
    # better not use DIAG_WORLDMODEL_SHARED, it is interpreted already
    (v, life) = _robotInterface.rtdbGet("LOCALIZATION_CANDIDATES")
    if v == None:
        return None
    if len(v) == 0:
        return None
    p = v[0]
    t = float(p[1][0] + 1e-6 * p[1][1])
    # TODO mirror? we might not always have same orientation (flip)
    return (t, RobotPose(p[2], p[3], p[4]))


def findClosestObstacle(x, y):
    pos = Vec2d(x, y)
    obstacles = _robotInterface.worldState.getObstacles()
    numObstacles = len(obstacles)
    if numObstacles == 0:
        return None
    result = None
    bestDist = 999
    for obst in obstacles:
        dist = (obst - pos).size()
        if dist < bestDist:
            result = obst
            bestDist = dist
    return result

def setConfig(module, argDict):
    _robotInterface.config.set(module, argDict)

def getConfig(module):
    return _robotInterface.config.get(module)

def restoreConfig(module):
    _robotInterface.config.restore(module)

def restoreAllConfigs():
    _robotInterface.config.restoreAll()


# install signal handler to be able to break from blocking actions
# TODO: linked to architecture redesign ...
"""
import signal
class KeyboardInterruptException(Exception):
    pass
def signal_handler(signal, frame):
    shutDown()
    raise KeyboardInterruptException()
signal.signal(signal.SIGINT, signal_handler)
"""
