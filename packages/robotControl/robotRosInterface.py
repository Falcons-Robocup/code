""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python library for commands, on-robot.
# This file implements the ROS topics- and service connections.
# 
# Implementations:
#    robotCLI.py          # command-line interface
#    robotLibrary.py      # discloses all basic commands and scenarios, also includes parser
#    robotScenarios.py    # scenario interface
#    robotRosInterface.py # basic commands and their ROS interfaces
#    scenarios/*.py       # scenario implementations
# 
# Jan Feitsma, 2016-12-17



# generic handy utilities
from time import sleep, time, strftime
import datetime
import threading
from math import pi, sqrt, atan2, sin, cos
from numpy import sign
import os

# ROS modules
import roslib
import rospy
import rosgraph
import dynamic_reconfigure.client

# Falcons modules
roslib.load_manifest('rosMsgs')
roslib.load_manifest('worldModel')
roslib.load_manifest('teamplay')
roslib.load_manifest('pathPlanning')
roslib.load_manifest('peripheralsInterface')
from worldModel.msg import t_wmInfo
from teamplay.srv import s_tp_input_command, s_tp_input_commandRequest
from pathPlanning.srv import s_pathplanning_move_while_turning, s_pathplanning_move_while_turningRequest
from peripheralsInterface.srv import *
from rosMsgs.msg import t_robotspeed, t_target, BallPossession, t_event
from FalconsTrace import trace
import FalconsEnv
from FalconsCoordinates import Position2D
import FalconsTrace

# one object in this module
_rosInterface = None
_verbose = 0
    


class RobotRosInterface():
    def __init__(self, robotId):
        self.robotId = robotId
        self.prefix = "/teamA/robot" + str(robotId) + "/"

        # Redirect tracing, do not overwrite
        FalconsTrace._trace_file_name_override = "trace_%s%d_commands.txt" % (FalconsEnv.get_team_name()[-1], robotId)
        FalconsTrace._trace_file_mode = "a"
        
        # Check if ROS master is running.
        # The next command (init_node) has no timeout, so there is no graceful error handling possible.
        # Therefore check if /rostopic exists.
        try:
            rosgraph.Master('/rostopic').getPid()
        except:
            raise Exception("ROS appears to be offline...")

        # Init ROS node
        rospy.init_node("robotRosInterface" + str(robotId), anonymous=True)

        # Init Teamplay override service
        try:
            rospy.wait_for_service(self.prefix + "s_tp_input_command", timeout=2)
        except:
            raise Exception("robot %d software appears to be offline..." % (robotId))
        self.tpservice = rospy.ServiceProxy(self.prefix + "s_tp_input_command", s_tp_input_command, True)

        # Init kicker services
        self.kickSpeedService = rospy.ServiceProxy(self.prefix + "s_kick_speed", s_peripheralsInterface_setKickSpeed, True)
        self.kickHeightService = rospy.ServiceProxy(self.prefix + "s_kick_position", s_peripheralsInterface_setKickPosition, True)
        
        # Simulated?
        hostname = os.uname()[1]
        self.isSimulated = ("FALCON" or "TURTLE" in hostname)

        # Init ballhandler enable/disable service
        self.hasBallHandlers = (robotId != 1)
        if self.hasBallHandlers:
            self.bhEnableService = rospy.ServiceProxy(self.prefix + "s_enable_ballHandlers", s_enable_ballHandlers, True)
            self.bhDisableService = rospy.ServiceProxy(self.prefix + "s_disable_ballHandlers", s_disable_ballHandlers, True)
        
        # Init WorldModel subscriber
        self.wmMsg = t_wmInfo()
        self.wmsubscriber = rospy.Subscriber(self.prefix + "t_wmInfo", t_wmInfo, self.cbWorldModel)
        
        # Init topic for diagnostics event, to relay output to visualizer + bag
        self.publish_event_topic = rospy.Publisher(self.prefix + "g_event", t_event, queue_size=10)

        # Init topic to publish velocity setpoint
        self.publish_velocity_topic = rospy.Publisher(self.prefix + "g_robotspeed", t_robotspeed, queue_size=10)

        # Init topic to publish position setpoint
        self.publish_position_topic = rospy.Publisher(self.prefix + "g_target", t_target, queue_size=10)

        # Init blocking pathplanning service
        self.targetPosService = rospy.ServiceProxy(self.prefix + "s_pathplanning_move_while_turning", s_pathplanning_move_while_turning, True)
        
        # Init configurators
        self.configClient = {}
        self.configClient["worldModelNode/localization"] = dynamic_reconfigure.client.Client(self.prefix + "worldModelNode/localization", timeout=2)
        self.configClient["worldModelNode/obstacleTracker"] = dynamic_reconfigure.client.Client(self.prefix + "worldModelNode/obstacleTracker", timeout=2)
        self.configClient["PathPlanningNode"] = dynamic_reconfigure.client.Client(self.prefix + "PathPlanningNode", timeout=2)
        try:
            self.configClient["peripheralsInterface/motors"] = dynamic_reconfigure.client.Client(self.prefix + "peripheralsInterface/motors", timeout=2)
        except:
            # apparently we are in simulation mode (!)
            pass

        # Always enable obstacle avoidance since robotCLI is used for debug and setup mode
        self.configClient["PathPlanningNode"].update_configuration({"Obstacle_avoidance_enabled": True})

        # Store initial configurations, so some scenarios can easily reset
        self.configDefault = {}
        for module in self.configClient.keys():
            self.configDefault[module] = self.configClient[module].get_configuration(timeout=2)
        
        # Sleep to give ROS some time to initialize ....... take this out and scenarioBhSpeedX will never succeed in first move
        sleep(1)

    def cbWorldModel(self, msg):
        self.wmMsg = msg

def setVerbose(level):
    global _verbose
    _verbose = level

def generateEvent(eventString, eventType=0):
    event = t_event()
    event.robotId = _rosInterface.robotId
    event.timeStamp = time() - 1388534400 # TODO move to falconsCommon, see also analyzerRealtime.py
    event.fileName = "robotCLI.py" # details not relevant, would need stack introspection like in tracing to figure this out
    event.eventType = eventType
    event.eventString = eventString
    _rosInterface.publish_event_topic.publish(event)

def log(msg, level=1, eventType=0):
    trace("%s", msg) # note: msg may contain a percentage
    if level <= _verbose:
        timestamp = (str(datetime.datetime.now()).replace(' ', ','))
        logmsg = timestamp + " : " + msg
        print logmsg
        # extra: relay level0 messages to coach, so the bag is selfcontaining
        if level == 0:
            generateEvent(msg, eventType)
    

#TODO
#def guessRobotId():
# simulator: check which robots are active

def isSimulated():
    return _rosInterface.isSimulated

# entry point for clients
def constructRosInterface(robotId):
    global _rosInterface
    _rosInterface = RobotRosInterface(robotId)

def setSpeed(vx, vy, vphi):
    """
    Send a single speed setpoint (RCS) to peripheralInterface.
    """
    setpointMsg = t_robotspeed()
    setpointMsg.vx = vx
    setpointMsg.vy = vy
    setpointMsg.vphi = vphi
    _rosInterface.publish_velocity_topic.publish(setpointMsg)

def setTarget(x, y, phi=0):
    """
    Send a single target setpoint (FCS) to pathPlanning.
    """
    targetMsg = t_target()
    targetMsg.x = x
    targetMsg.y = y
    targetMsg.phi = phi
    _rosInterface.publish_position_topic.publish(targetMsg)

def setTargetBlocking(x, y, phi=0, motionProfile=t_target.MOTIONPROFILE_NORMAL):
    """
    Send a blocking target setpoint (FCS) to pathPlanning.
    """
    targetPosMsg = s_pathplanning_move_while_turningRequest()
    targetPosMsg.pos.x = x
    targetPosMsg.pos.y = y
    targetPosMsg.pos.theta = phi
    targetPosMsg.motion_profile = motionProfile
    # Blocking interface call to PathPlanning to move to position (x, y, phi)
    _rosInterface.targetPosService(targetPosMsg)
        
def calculateDeltaPhi(phi1, phi2=None):
    """
    Calculate delta angle with a sign, such that it is between -pi and pi.
    If second argument is omitted, then current angle is taken.
    """
    if phi2 == None:
        phi2 = _rosInterface.wmMsg.locationTheta
    dPhi = phi1 - phi2
    while dPhi >= pi:
        dPhi -= 2*pi
    while dPhi <= -pi:
        dPhi += 2*pi
    return dPhi

def calculateTargetPhi(x, y):
    """
    Calculate target angle w.r.t. current angle.
    """
    robotX = _rosInterface.wmMsg.locationX
    robotY = _rosInterface.wmMsg.locationY
    robotPhi = _rosInterface.wmMsg.locationTheta
    return atan2(y - robotY, x - robotX)

def enableBallHandlers():
    if _rosInterface.hasBallHandlers:
        msg = s_enable_ballHandlersRequest()
        response = _rosInterface.bhEnableService(msg)
        log("enabled ballHandlers")

def disableBallHandlers():
    if _rosInterface.hasBallHandlers:
        msg = s_disable_ballHandlersRequest()
        response = _rosInterface.bhDisableService(msg)
        log("disabled ballHandlers")

def safeKick(power=30, height=0):
    if hasBall():
        kick(power, height)
        
def kick(power=30, height=0):
    log(">kick power=%6.2f height=%6.2f" % (power, height))
    # TODO: rather use shootPlanning
    msg = s_peripheralsInterface_setKickPositionRequest()
    msg.kick_position = height
    response = _rosInterface.kickHeightService(msg)
    sleep(1) # give stepper motor some time to adjust
    msg = s_peripheralsInterface_setKickSpeedRequest()
    msg.kick_speed = power
    response = _rosInterface.kickSpeedService(msg)
    # set kicker height back to 0, if needed
    if height > 0:
        msg = s_peripheralsInterface_setKickPositionRequest()
        msg.kick_position = 0
        response = _rosInterface.kickHeightService(msg)
    log("<kick")
    
def tpControl(command):
    #log(">tpControl command='%s'" % (command))
    msg = s_tp_input_commandRequest()
    msg.command = command
    response = _rosInterface.tpservice(msg)
    #log("<tpControl response='%s'" % (response))
    return response.result

def getPosition(robotId=None):
    # default to own position
    if robotId == None or robotId == _rosInterface.robotId:
        return ownPosition()
    # teammember position of specified robotId
    idx = _rosInterface.wmMsg.teamMemberRobotID.index(chr(robotId))
    robotX = _rosInterface.wmMsg.teamMemberX[idx]
    robotY = _rosInterface.wmMsg.teamMemberY[idx]
    robotPhi = _rosInterface.wmMsg.teamMemberTheta[idx]
    return (robotX, robotY, robotPhi)

def getVelocity(robotId=None):
    # default to own velocity
    if robotId == None or robotId == _rosInterface.robotId:
        return ownSpeed() # TODO rename to ownVelocity, speed is normally a scalar
    # teammember position of specified robotId
    idx = _rosInterface.wmMsg.teamMemberRobotID.index(chr(robotId))
    robotVx = _rosInterface.wmMsg.teamMemberVx[idx]
    robotVy = _rosInterface.wmMsg.teamMemberVy[idx]
    robotVphi = _rosInterface.wmMsg.teamMemberVtheta[idx]
    return (robotVx, robotVy, robotVphi)

def ownPosition():
    return (_rosInterface.wmMsg.locationX, _rosInterface.wmMsg.locationY, _rosInterface.wmMsg.locationTheta)
    
def ownSpeed():
    return (_rosInterface.wmMsg.locationVx, _rosInterface.wmMsg.locationVy, _rosInterface.wmMsg.locationVtheta)
    
def ballSpeed():
    return (_rosInterface.wmMsg.ballVX, _rosInterface.wmMsg.ballVY, _rosInterface.wmMsg.ballVZ)
    
def ballPosition():
    return (_rosInterface.wmMsg.ballX, _rosInterface.wmMsg.ballY, _rosInterface.wmMsg.ballZ)

def teamSeesBall():
    return _rosInterface.wmMsg.isBallValid
    
def ballOnSameHalf():
    if teamSeesBall():
        ownY = ownPosition()[1]
        ballY = ballPosition()[1]
        return ownY * ballY > 0 # check if same sign
    return False
    
def robotId():
    return _rosInterface.robotId
    
def robotCloseBy(x, y, phi=None, xyTol=0.1, phiTol=0.05):
    robotX = _rosInterface.wmMsg.locationX
    robotY = _rosInterface.wmMsg.locationY
    dr2 = (robotX - x)**2 + (robotY - y)**2
    xyOk = dr2 < xyTol**2
    dPhi = calculateDeltaPhi(_rosInterface.wmMsg.locationTheta, phi)
    phiOk = True
    # check phi?
    if phi != None:
        phiOk = abs(dPhi) < phiTol
    log(" dXY=%6.2f dPhi=%6.2f xyOk=%d phiOk=%d" % (sqrt(dr2), dPhi, xyOk, phiOk), 2)
    return xyOk and phiOk

def ballCloseBy(x, y, xyTol=0.1):
    ballX = _rosInterface.wmMsg.ballX
    ballY = _rosInterface.wmMsg.ballY
    return (ballX - x)**2 + (ballY - y)**2 < xyTol**2

def findClosestObstacle(robotPos):
    m = _rosInterface.wmMsg
    numObstacles = m.nrObstacleMeasurements
    assert(numObstacles > 0)
    result = None
    bestDistSquared = 999
    for iObst in range(numObstacles):
        distSquared = (m.obstacleX[iObst] - robotPos[0])**2 + (m.obstacleY[iObst] - robotPos[1])**2
        if distSquared < bestDistSquared:
            result = ((m.obstacleX[iObst], m.obstacleY[iObst]), (m.obstacleVX[iObst], m.obstacleVY[iObst]))
    return result

def startThread(f, *args):
    t = threading.Thread(target=f, args=args)
    t.start()
        
def activeRobots():
    L = [ord(x) for x in _rosInterface.wmMsg.activeRobots] # activeRobots is an uint8 array, which in python effectively is a string...
    return L
    
def myRelIndex():
    return list(activeRobots()).index(_rosInterface.robotId)

def myRobotId():
    return int(_rosInterface.robotId)

def hasBall(robotId = None):
    if robotId == None:
        return (_rosInterface.wmMsg.possession.type == BallPossession.TYPE_TEAMMEMBER) and (_rosInterface.wmMsg.possession.robotID == _rosInterface.robotId)
    # check specific robotId
    return (_rosInterface.wmMsg.possession.type == BallPossession.TYPE_TEAMMEMBER) and (_rosInterface.wmMsg.possession.robotID == robotId)
    
def setConfig(module, argDict):
    _rosInterface.configClient[module].update_configuration(argDict)

def restoreConfig(module):
    _rosInterface.configClient[module].update_configuration(_rosInterface.configDefault[module])

def restoreAllConfigs():
    for module in _rosInterface.configClient.keys():
        _rosInterface.configClient[module].update_configuration(_rosInterface.configDefault[module])

# install signal handler to be able to break from blocking actions
import signal
class KeyboardInterruptException(Exception):
    pass
def signal_handler(signal, frame):
    raise KeyboardInterruptException()
signal.signal(signal.SIGINT, signal_handler)

