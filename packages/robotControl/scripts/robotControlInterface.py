""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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

    def clearStimulation(self):
        self._robotControl.clearStimulation()


    ####################################
    # Public Teamplay helper functions #
    ####################################
    
    # Example: rci.setTeamplayGameState("IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL", {})
    # Reference: sharedTypes.py => treeEnum
    def setTeamplayGameState(self, gsName, params):
        # find tree enum value belonging to given name
        treeValue = sharedTypes.treeEnum[gsName].value
        self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.GAMESTATE.value, treeEnumValue=treeValue, tpActionEnumValue=sharedTypes.tpActionEnum.INVALID.value, params=params)

    # Example: rci.setTeamplayRole("ATTACKER_MAIN", {'role': "attackerMain"})
    # Reference: sharedTypes.py => treeEnum
    def setTeamplayRole(self, roleName, params):
        # find tree enum value belonging to given name
        treeValue = sharedTypes.treeEnum[roleName].value
        self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.ROLE.value, treeEnumValue=treeValue, tpActionEnumValue=sharedTypes.tpActionEnum.INVALID.value, params=params)

    # Example: rci.setTeamplayBehavior("SEARCH_BALL", {'role': "attackerMain"})
    # Reference: sharedTypes.py => treeEnum
    def setTeamplayBehavior(self, behName, params):
        # find tree enum value belonging to given name
        treeValue = sharedTypes.treeEnum[behName].value
        self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.BEHAVIOR.value, treeEnumValue=treeValue, tpActionEnumValue=sharedTypes.tpActionEnum.INVALID.value, params=params)

    # Example: rci.setTeamplayAction("GET_BALL", {})
    # Example: rci.setTeamplayAction("MOVE", {"target": "coord:4.5,6.8"})
    # Reference: sharedTypes.py => tpActionEnum
    def setTeamplayAction(self, tpActionName, params):
        # find tpAction enum value belonging to given name
        tpActionEnumValue = sharedTypes.tpActionEnum[tpActionName].value
        self.setTeamplayOverride(active=True, tpOverrideLevelEnumValue=sharedTypes.tpOverrideLevelEnum.TP_ACTION.value, treeEnumValue=sharedTypes.treeEnum.INVALID.value, tpActionEnumValue=tpActionEnumValue, params=params)


    ##############################
    # Public Interface functions #
    ##############################

    # Simple sleep.
    def sleep(self, duration):
        # leave bhEnabled intact
        bhEnabled = self.getBallHandlersEnabled()
        self.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, False, bhEnabled)
        self._robotControl.sleep(duration)

    # See the teamplay actions above.
    def setTeamplayOverride(self, active, tpOverrideLevelEnumValue, treeEnumValue, tpActionEnumValue, params):
        self._robotControl.stimulate([("TP_OVERRIDE_STATE", {'active': active, 'level': tpOverrideLevelEnumValue, 'treeValue': treeEnumValue, 'tpAction': tpActionEnumValue, 'params': params}), ("TP_HEARTBEAT", 0)])

    # motionPlanning actions are overruled via teamplay, as motionPlanning is implemented as a library
    # Example: rci.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, False, False)
    # Example: rci.setMotionPlanningAction("MOVE", 0.0, 0.0, 0.0, False, True)
    # Example: rci.setMotionPlanningAction("KICK", power, height, 0.0, False, True)
    # Reference: sharedTypes => actionTypeEnum
    def setMotionPlanningAction(self, actionTypeName, x, y, z, slow, ballHandlersEnabled):
        # find actionTypeEnum value belonging to given name
        actionTypeEnumValue = sharedTypes.actionTypeEnum[actionTypeName].value
        self._robotControl.stimulate([("TP_OVERRIDE_STATE", {'active': True, 'level': sharedTypes.tpOverrideLevelEnum.MP_ACTION.value, 'treeValue': sharedTypes.treeEnum.INVALID.value, 'tpAction': sharedTypes.tpActionEnum.INVALID.value, 'params': {}, 'mpAction': {'action': actionTypeEnumValue, 'position': [x, y, z], 'slow': slow, 'ballHandlersEnabled': ballHandlersEnabled}}), ("TP_HEARTBEAT", 0)])

    # DEPRECATED - a PathPlanning move is now triggered using a MotionPlanning move.
    ## Example: rci.setPathPlanningMoveSetpoint("MOVE", 0.0, 0.0, 0.0, False)
    ## Example: rci.setPathPlanningMoveSetpoint("STOP", 0.0, 0.0, 0.0, False)
    #def setPathPlanningMoveSetpoint(self, actionTypeName, x, y, phi, slow):
    #    actionTypeEnumValue = sharedTypes.actionTypeEnum[actionTypeName].value
    #    self._robotControl.stimulate([("MOTION_SETPOINT", {'action': actionTypeEnumValue, 'position': [x, y, phi], 'slow': slow})])

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


