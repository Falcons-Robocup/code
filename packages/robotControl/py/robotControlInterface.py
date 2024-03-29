# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import sys
import falconspy
import sharedTypes

import robotControl

class RobotControlInterface():

    def __init__(self, robotId):
        self._robotControl = robotControl.RobotControl(robotId)
        self._robotId = robotId

    ####################
    # Public functions #
    ####################

    def connect(self):
        self._robotControl.connect()

    def disconnect(self):
        self._robotControl.disconnect()

    def blockUntilTPOverridePassedOrFailed(self):
        self._robotControl.blockUntilTPOverridePassedOrFailed()

    def blockUntilMPPassedOrFailed(self):
        self._robotControl.blockUntilMPPassedOrFailed()

    def blockUntilVelocitySettled(self, settleTime):
        self._robotControl.blockUntilVelocitySettled(settleTime)

    def clearStimulation(self):
        self._robotControl.clearStimulation()


    ####################################
    # Public Teamplay helper functions #
    ####################################
    
    # Example: rci.setTeamplayGameState("IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL")
    def setTeamplayGameState(self, gsName):
        self.setTeamplayOverride(active=True, overrideStr=gsName)

    # Example: rci.setTeamplayRole("ATTACKER_MAIN")
    def setTeamplayRole(self, roleName):
        self.setTeamplayOverride(active=True, overrideStr=roleName)

    #####
    # 2021-06-25 EKPC -- Behavior and Action not (yet?) supported 
    #####
    ## Example: rci.setTeamplayBehavior("SEARCH_BALL", {'role': "attackerMain"})
    ## Reference: sharedTypes.py => treeEnum
    #def setTeamplayBehavior(self, behName, params):
    #    # find tree enum value belonging to given name
    #    treeValue = sharedTypes.treeEnum[behName].value
    #    self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.BEHAVIOR.value, treeEnumValue=treeValue, tpActionEnumValue=sharedTypes.tpActionEnum.INVALID.value, params=params)

    ## Example: rci.setTeamplayAction("GET_BALL", {})
    ## Example: rci.setTeamplayAction("MOVE", {"target": "coord:4.5,6.8"})
    ## Reference: sharedTypes.py => tpActionEnum
    #def setTeamplayAction(self, tpActionName, params):
    #    # find tpAction enum value belonging to given name
    #    tpActionEnumValue = sharedTypes.tpActionEnum[tpActionName].value
    #    self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.TP_ACTION.value, treeEnumValue=sharedTypes.treeEnum.INVALID.value, tpActionEnumValue=tpActionEnumValue, params=params)


    ##############################
    # Public Interface functions #
    ##############################

    # Simple sleep.
    def sleep(self, duration):
        # leave bhEnabled intact
        bhEnabled = self.getBallHandlersEnabled()
        self.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, "NORMAL", bhEnabled)
        self._robotControl.sleep(duration)

    # See the teamplay actions above.
    def setTeamplayOverride(self, active, overrideStr):
        self._robotControl.stimulate([("TP_OVERRIDE_STATE", {'active': active, 'overrideStr': overrideStr}), ("TP_HEARTBEAT", 0)])

    # Example: rci.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, "NORMAL", False)
    # Example: rci.setMotionPlanningAction("MOVE", 0.0, 0.0, 0.0, "NORMAL", True)
    # Example: rci.setMotionPlanningAction("KICK", power, height, 0.0, "NORMAL", True)
    # Reference: sharedTypes => actionTypeEnum
    def setMotionPlanningAction(self, actionTypeName, x, y, z, motionTypeName, ballHandlersEnabled):
        actionTypeEnumValue = sharedTypes.actionTypeEnum[actionTypeName].value
        motionTypeEnumValue = sharedTypes.motionTypeEnum[motionTypeName].value
        self._robotControl.stimulate([('ACTION', {'action': actionTypeEnumValue, 'position': [x, y, z], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': ballHandlersEnabled})])

    # DEPRECATED - a PathPlanning move is now triggered using a MotionPlanning move.
    ## Example: rci.setPathPlanningMoveSetpoint("MOVE", 0.0, 0.0, 0.0, False)
    ## Example: rci.setPathPlanningMoveSetpoint("STOP", 0.0, 0.0, 0.0, False)
    #def setPathPlanningMoveSetpoint(self, actionTypeName, x, y, phi, slow):
    #    actionTypeEnumValue = sharedTypes.actionTypeEnum[actionTypeName].value
    #    self._robotControl.stimulate([("MOTION_SETPOINT", {'action': actionTypeEnumValue, 'position': [x, y, phi], 'slow': slow})])

    # Example: rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
    def setRobotPosVel(self, robotPosVelName, x, y, rz, vx, vy, vrz, motionTypeName):
        robotPosVelEnumValue = sharedTypes.robotPosVelEnum[robotPosVelName].value
        motionTypeEnumValue = sharedTypes.motionTypeEnum[motionTypeName].value
        self._robotControl.stimulate([("ROBOT_POSVEL_SETPOINT", [robotPosVelEnumValue, [x, y, rz], [vx, vy, vrz], motionTypeEnumValue])])

    # Example: rci.setRobotVelocity(0.0, 0.0, 1.0)
    def setRobotVelocity(self, vx, vy, vphi):
        self._robotControl.stimulate([("ROBOT_VELOCITY_SETPOINT", [vx, vy, vphi])])

    # Example: rci.setMotorsVelocity(0.0, 0.0, 0.0)
    def setMotorsVelocity(self, vm1, vm2, vm3):
        self._robotControl.stimulate([("MOTOR_VELOCITY_SETPOINT", [vm1, vm2, vm3])])

    # Example: rci.setShootPlanningShootSetpoint("PREPARE", "LOB", 0.0, 0.0, 0.0)
    # Example: rci.setShootPlanningShootSetpoint("SHOOT", "LOB", 0.0, 0.0, 0.0)
    def setShootPlanningShootSetpoint(self, shootPhaseName, shootTypeName, x, y, z):
        shootPhaseEnumValue = sharedTypes.shootPhaseEnum[shootPhaseName].value
        shootTypeEnumValue = sharedTypes.shootTypeEnum[shootTypeName].value
        self._robotControl.stimulateOnce([("SHOOT_SETPOINT", {'shootPhase': shootPhaseEnumValue, 'shootType': shootTypeEnumValue, 'position': [x, y, z]})])

    # BROKEN WHILE TESTING. Not sure yet why.
    # rci.setKickerSetpoint("SET_HEIGHT", 100.0, 0.0)
    def setKickerSetpoint(self, kickerSetpointName, kickerHeight, kickerPower):
        kickerSetpointTypeEnumValue = sharedTypes.kickerSetpointTypeEnum[kickerSetpointName].value
        self._robotControl.stimulateOnce([("KICKER_SETPOINT", {'kickerSetpointType': kickerSetpointTypeEnumValue, 'kickerHeight': kickerHeight, 'kickerPower': kickerPower})])

    # Example: rci.setBallHandlers(True)
    def setBallHandlers(self, enabled):
        self._robotControl.stimulate([("BALLHANDLERS_SETPOINT", enabled)])
    def getBallHandlersEnabled(self):
        r = self._robotControl._rtdbGet("BALLHANDLERS_SETPOINT", timeout=None)
        if r != None:
            return r.value
        return False

    # Unsupported. Part of the feedback loop, and therefore always overwritten, even during TEST_MODE
    #def setBallHandlerAngleAndVelocity(self, enabled, angleLeft, angleRight, velocityLeft, velocityRight):
    #    self._robotControl.stimulate([("BALLHANDLERS_MOTOR_SETPOINT", {'enabled': enabled, 'bhMotorData': {'angleLeft': angleLeft, 'angleRight': angleRight, 'velocityLeft': velocityLeft, 'velocityRight': velocityRight}})])

    # Example: rci.setKeeperFrameExtend("LEFT")
    # Example: rci.setKeeperFrameExtend("RIGHT")
    # Example: rci.setKeeperFrameExtend("UP")
    def setKeeperFrameExtend(self, keeperFrameSetpointName):
        keeperFrameSetpointEnumValue = sharedTypes.keeperFrameSetpointEnum[keeperFrameSetpointName].value
        self._robotControl.stimulateOnce([("KEEPERFRAME_SETPOINT", keeperFrameSetpointEnumValue)])


