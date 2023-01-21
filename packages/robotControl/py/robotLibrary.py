# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Python library for on-robot commands.



import time
import math
import inspect

import falconspy
import falconsrtdb
import robotControlInterface
import worldState

# Import FalconsCoordinates for use in Blockly code
from FalconsCoordinates import *

# load all scenarios dynamically from the folder 'scenarios', which has a special __init.py__
import scenarios



class RobotLibrary():

    def __init__(self, robotId):
        self._robotId = robotId

    ##################
    # Init functions #
    ##################

    def connect(self):
        """
        Setup the robot interface.
        """
        # WorldState has a potential race condition:
        # It will update its administration in a thread.
        # If the administration is accessed before the administration is updated, it throws an error.
        # Beware: RobotControlInterface has a sleep in its connect() which counters this race condition.
        self._ws = worldState.WorldState(self._robotId)
        self._ws.startMonitoring()
        self._rci = robotControlInterface.RobotControlInterface(self._robotId)
        self._rci.connect()


    def disconnect(self):
        """
        Cleanup such that robot is in a good state to play (refbox).
        """
        self._rci.disconnect()
        self._ws.stopMonitoring()


    def isInMatchMode(self):
        """
        Returns True if MATCH_MODE is enabled
        Returns False otherwise
        """
        rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=True)
        rtdbStore.refresh_rtdb_instances()
        result = rtdbStore.get(self._robotId, "MATCH_MODE", timeout=None).value
        rtdbStore.closeAll()
        return result


    ####################
    # Action functions #
    ####################

    def stop(self, keep_bh=True):
        # to ensure robot does not drop the ball right after its action finished and a STOP is issued,
        # default behavior (keep_bh=True) is to maintain ball handler setpoint
        # to overrule this, set keep_bh=False and the robot will drop the ball upon this STOP command
        bh_enabled = False
        if keep_bh:
            bh_enabled = self._rci.getBallHandlersEnabled()
        self._rci.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, "NORMAL", bh_enabled)
        self._rci.clearStimulation()
        self._rci.setRobotVelocity(0.0, 0.0, 0.0)

    def enableBallHandlers(self):
        self._rci.setBallHandlers(True)

    def disableBallHandlers(self):
        self._rci.setBallHandlers(False)

    def hasBall(self):
        return self._ws.hasBall()

    def seesBall(self):
        return self._ws.getBallPosition() != None

    def velocitySetpoint(self, vx, vy, vphi, duration):
        self._rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, vx, vy, vphi, "NORMAL")
        # next line was old style, before VC was split out of PP, so no acc limiter:
        #self._rci.setRobotVelocity(0.0, 0.0, 0.0)
        time.sleep(duration)
        self.stop()

    def sleep(self, duration):
        time.sleep(duration)
    
    def move(self, x, y, rz=None, motionType="NORMAL"):
        # TODO: is there still a use case to overrule (pp) tolerance limits?
        # pathPlanning tolerances might be configured a bit too tight for smooth testing
        # if rotation is unspecified, then just face away from current pose, ending up in forward-drive pose, no extra rotation afterwards
        if rz == None:
            pos = self._ws.getRobotPosition()
            rz = math.atan2(y - pos.y, x - pos.x)
        bh = self._rci.getBallHandlersEnabled() # maintain current state
        self._rci.setMotionPlanningAction("MOVE", x, y, rz, motionType, bh)
        self._rci.blockUntilMPPassedOrFailed()
        self.stop()

    def getBall(self):
        self._rci.setMotionPlanningAction("GET_BALL", 0.0, 0.0, 0.0, "NORMAL", True)
        self._rci.blockUntilMPPassedOrFailed()
        self.stop()

    def ballCloseBy(self, distance=0.5, x=None, y=None):
        bpos = self._ws.getBallPosition()
        if bpos == None:
            return False
        if x == None or y == None:
            pos = self._ws.getRobotPosition()
            x = pos.x
            y = pos.y
        result = (Vec2d(bpos.x, bpos.y) - Vec2d(x, y)).size() < distance
        return result

    def faceTowards(self, x, y):
        pos = self._ws.getRobotPosition()
        robotX = pos.x
        robotY = pos.y
        targetRz = math.atan2(y - pos.y, x - pos.x)
        target = (pos.x, pos.y, targetRz)
        self.move(*target)
        return target
    
    def interceptBall(self):
        self._rci.setMotionPlanningAction("INTERCEPT_BALL", 0.0, 0.0, 0.0, "NORMAL", True)
        success = self._rci.blockUntilMPPassedOrFailed()
        if success:
            self.stop()
            # in case of intercept action failure, it would not be wise to call STOP
            # this would make the robot stutter-strafe because SPG velocity setpoint resets

    def passTo(self, x, y):
        self._rci.setMotionPlanningAction("PASS", x, y, 0.0, "NORMAL", True)
        self._rci.blockUntilMPPassedOrFailed()
        self.stop()

    def shootAt(self, x, y, z):
        self._rci.setMotionPlanningAction("SHOOT", x, y, z, "NORMAL", True)
        self._rci.blockUntilMPPassedOrFailed()
        self.stop()

    def lobShotAt(self, x, y, z):
        self._rci.setMotionPlanningAction("LOB", x, y, z, "NORMAL", True)
        self._rci.blockUntilMPPassedOrFailed()
        self.stop()

    def kick(self, power, height=0.0):
        self._rci.setMotionPlanningAction("KICK", power, height, 0.0, "NORMAL", True)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self.stop()

    def keeper(self):
        self.behavior("B_GOALKEEPER")

    def behavior(self, behaviorName, params={}):
        self._rci.setTeamplayBehavior(args[0], params)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    ######################
    # Scenario functions #
    ######################

    ## private

    def _scenarioList(self):
        """
        Return the dynamically loaded list of scenario functions (as strings).
        Use _scenarioGet to get the function object.
        """
        result = []
        for f in scenarios.__all__:
            lbd = lambda f: inspect.isfunction(f) or inspect.isclass(f)
            result += [m[0] for m in inspect.getmembers(getattr(scenarios, f), lbd) if "scenarios." in m[1].__module__ and m[0][0] != '_']
        return result

    def _scenarioGet(self, name):
        """
        Return the function which belongs to given name, for example 'driveCircle' or 'intercept'.
        """
        if not name in self._scenarioList():
            raise Exception("Error: Unrecognized scenario '%s'" % (name))
        scenarioFunction = None
        for f in scenarios.__all__:
            if hasattr(getattr(scenarios, f), name):
                return getattr(getattr(scenarios, f), name)
        raise Exception("Error: Unable to find scenario '%s'" % (name))

    ## public

    def scenarioInfo(self):
        """
        Print information for all scenario. Just dump the __doc__ string of the functions/classes.
        """
        for s in sorted(self._scenarioList()):
            print('Scenario: ' + s)
            print(self._scenarioGet(s).__doc__)


    def scenarioRun(self, name, *args):
        """
        Execute given scenario with optional arguments.
        If called from command line, then arguments will all be strings. Scenario functions must sanitize inputs.
        """
        # check the dynamically loaded list of scenarios
        scenarioFunction = self._scenarioGet(name)
        # call it
        scenarioFunction(self, *args)


    def scenarioRunFromString(self, scenarioStr):
        """
        Execute scenario code in `scenarioStr`
        """

        myRobotId = self._robotId

        # Import simScene for TeleportBall in Blockly codd
        import simScene

        # Import EnvironmentField for use in Blockly code
        import EnvironmentField

        # robotControlInterface and worldState are used by Blockly code
        rci = self._rci
        ws = self._ws

        # Execute the scenario
        exec(scenarioStr)


