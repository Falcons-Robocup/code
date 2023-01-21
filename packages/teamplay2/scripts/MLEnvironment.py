# Copyright 2021-2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
# MLEnvironment.py
#
# Date: 2020-06-19
# Author: Erik Kouters
#

import time
import threading
import subprocess

import gym
import numpy as np

import falconspy
import falconsrtdb
import robotControlInterface
import worldState   # import worldState from worldModel
import sharedTypes
import EnvironmentField
from FalconsCoordinates import *

# simworld publishes ROBOT_STATE
# heartBeat connects ROBOT_STATE to TP_HEARTBEAT
# teamplay ticks on TP_HEARTBEAT

# Connect RobotInterface. Then HB no longer connects ROBOT_STATE to TP_HEARTBEAT.
# simworld publishes ROBOT_STATE
# here, we waitForPut(ROBOT_STATE)


# We need a way to control the simulation from here.
# When to step.

# When executing an action from here, we publish on ACTION, and wait for ROBOT_VELOCITY_SETPOINT.
# How do control the simulation steps in that loop?

# In this order:
# - execute action (ACTION --> ROBOT_VELOCITY_SETPOINT)
# - step simulation (HEARTBEAT --> ROBOT_STATE)
# - compute reward
# - observe new world state (worldState.update())
# - decide episode_over



class SimulationEnvironment(gym.Env):

    def __init__(self, robotId):

        self.robotId = robotId

        self._rci = robotControlInterface.RobotControlInterface( robotId )
        self._rci.connect()

        self.ws = worldState.WorldState( robotId )

        self.rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=False) # write mode

        # Ensure simulation does not tick by itself
        self._pause_simulation()

        # - GetBall()
        # - Goalkeeper()
        # - InterceptBall()
        # TODO Dribble()
        # TODO MoveToFreeSpot()
        # TODO DefendAttackingOpponent()
        # - ShootAtGoal()
        # TODO PassToTeammember( int teammemberId ) // if teammemberId does not exist, does nothing
        # - TurnAwayFromOpponent()
        self.action_space = gym.spaces.Discrete(5)

        # Observation Space:
        #   Robot.x
        #   Robot.y
        #   Robot.Rz [0-2pi]
        #   Ball.x
        #   Ball.y
        #   RobotHasBall
        #   Dist(Robot, Ball)
        #   Angle(Robot.Rz, Ball)
        #   Dist(Robot, Goal)
        #   Angle(Robot.Rz, GoalCenter)
        #   Opening angle Goal
        #   Opening angle Teammember i (Ti) TODO
        #   Angle(Robot.Rz, Ti) TODO
        # Num values: 6 + 2T where T is num teammembers
        maxX = EnvironmentField.cEnvironmentField.getInstance().getWidth() / 2.0
        minX = -maxX
        maxY = EnvironmentField.cEnvironmentField.getInstance().getLength() / 2.0
        minY = -maxY

        # maxDist == field diagonal == Sqrt( fieldWidth^2 + fieldLength^2 )
        maxDist = math.sqrt( math.pow(EnvironmentField.cEnvironmentField.getInstance().getWidth(),2) + math.pow(EnvironmentField.cEnvironmentField.getInstance().getLength(),2) )

        observation_lower_bound = np.array([minX, minY, 0.0, minX, minY, 0, 0.0, 0.0, 0.0, 0.0, 0.0])
        observation_upper_bound = np.array([maxX, maxY, 2*math.pi, maxX, maxY, 1, maxDist, 2*math.pi, maxDist, 2*math.pi, 2*math.pi])
        self.observation_space = gym.spaces.Box(low=observation_lower_bound, high=observation_upper_bound, dtype=np.float32)

    def __del__(self):
        self._rci.disconnect()

    def runEpisode(self):
        self.reset()
        stop_episode = False

        idx = 0
        total_reward = 0.0
        while not stop_episode:
            action = self.action_space.sample()
            observation, reward, stop_episode, info = self.step( action )
            total_reward += reward
            print(idx, observation, reward, stop_episode, total_reward)
            idx += 1

        print("Episode done in", idx, "iterations")

        # Performs a stop robot
        self._rci.sleep(0.1)


    def _waitForPut(self, key):
        self.rtdbStore.waitForPut(self.robotId, key)


    def step(self, action):
        # 1. Execute Action
        # 2. Step Simulation
        # 3. Get Reward
        # 4. Get Observation
        # 5. Determine if episode is done
        # 6. Return [observation, reward, stop_episode, info]

        # 1.
        self._execute_action(action)

        # 2.
        self._step_simulation()

        # 3.
        reward = self._get_reward()

        # 4.
        obs = self._observe()

        # 5.
        stop_episode = self._stop_episode()

        return obs, reward, stop_episode, {}

    def _execute_action(self, action):
        # Assert valid action
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        # put ACTION
        # waitForPut ActionOutput (ROBOT_VELOCITY_SETPOINT or KICKER_SETPOINT)

        if action == 0:
            actionTypeEnumValue = sharedTypes.actionTypeEnum["GET_BALL"].value
            motionTypeEnumValue = sharedTypes.motionTypeEnum["NORMAL"].value
            #t = threading.Thread(target=self._waitForPut, args=("ROBOT_VELOCITY_SETPOINT",))
            t = threading.Thread(target=self._waitForPut, args=("ACTION_RESULT",))
            t.start()
            self._rci._robotControl.stimulateOnce([('ACTION', {'action': actionTypeEnumValue, 'position': [0.0, 0.0, 0.0], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': True})], do_sleep=False)
            t.join(timeout=0.5)
        elif action == 1:
            actionTypeEnumValue = sharedTypes.actionTypeEnum["KEEPER_MOVE"].value
            motionTypeEnumValue = sharedTypes.motionTypeEnum["NORMAL"].value
            #t = threading.Thread(target=self._waitForPut, args=("ROBOT_VELOCITY_SETPOINT",))
            t = threading.Thread(target=self._waitForPut, args=("ACTION_RESULT",))
            t.start()
            self._rci._robotControl.stimulateOnce([('ACTION', {'action': actionTypeEnumValue, 'position': [0.0, 0.0, 0.0], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': True})], do_sleep=False)
            t.join(timeout=0.5)
        elif action == 2:
            actionTypeEnumValue = sharedTypes.actionTypeEnum["INTERCEPT_BALL"].value
            motionTypeEnumValue = sharedTypes.motionTypeEnum["NORMAL"].value
            #t = threading.Thread(target=self._waitForPut, args=("ROBOT_VELOCITY_SETPOINT",))
            t = threading.Thread(target=self._waitForPut, args=("ACTION_RESULT",))
            t.start()
            self._rci._robotControl.stimulateOnce([('ACTION', {'action': actionTypeEnumValue, 'position': [0.0, 0.0, 0.0], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': True})], do_sleep=False)
            t.join(timeout=0.5)
        elif action == 3:
            actionTypeEnumValue = sharedTypes.actionTypeEnum["SHOOT"].value
            motionTypeEnumValue = sharedTypes.motionTypeEnum["NORMAL"].value
            #t = threading.Thread(target=self._waitForPut, args=("ROBOT_VELOCITY_SETPOINT",))
            t = threading.Thread(target=self._waitForPut, args=("ACTION_RESULT",))
            t.start()
            self._rci._robotControl.stimulateOnce([('ACTION', {'action': actionTypeEnumValue, 'position': [0.0, 9.0, 0.0], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': True})], do_sleep=False)
            t.join(timeout=0.5)
        elif action == 4:
            actionTypeEnumValue = sharedTypes.actionTypeEnum["TURN_AWAY_FROM_OPPONENT"].value
            motionTypeEnumValue = sharedTypes.motionTypeEnum["NORMAL"].value
            #t = threading.Thread(target=self._waitForPut, args=("ROBOT_VELOCITY_SETPOINT",))
            t = threading.Thread(target=self._waitForPut, args=("ACTION_RESULT",))
            t.start()
            self._rci._robotControl.stimulateOnce([('ACTION', {'action': actionTypeEnumValue, 'position': [0.0, 0.0, 0.0], 'motionType': motionTypeEnumValue, 'ballHandlersEnabled': True})], do_sleep=False)
            t.join(timeout=0.5)


    def _step_simulation(self):
        # tick simulation once: HEARTBEAT --> ROBOT_STATE
        t = threading.Thread(target=self._waitForPut, args=("ROBOT_STATE",))
        t.start()
        coachId = 0
        self.rtdbStore.put(coachId, "HEARTBEAT", 0)
        t.join(timeout=0.5)

        # after new ROBOT_STATE is written, update WorldState
        self.ws.update()

    def reset(self):
        # Load scene from a new process.
        # Rationale:
        #   simScene creates a new connection to RtDB.
        #   Each process can have only 1 RtDB connection, otherwise data goes corrupt.
        #   This process already has an RtDB connection, so importing and calling simScene within this process instantiates a second RtDB connection in this process
        subprocess.call("simScene.py MLTest.scene", shell=True)
        self.first_step = True
        self.had_ball = False

        # Step simulation once to ensure data (e.g., BALLS) is fresh
        # Otherwise data is too old, and rtdb.get returns STALE
        self._step_simulation()

        observation = self._observe()
        return observation


    def _get_reward(self):
        # How do we compute reward?

        # Gym Soccer: https://github.com/openai/gym-soccer
        # https://github.com/openai/gym-soccer/blob/dff241967ae0c1ea719d889cf9d88a37608944a2/gym_soccer/envs/soccer_empty_goal.py#L27
        # a) for approaching ball
        # b) for kicking ball  TODO
        # c) for scoring a goal TODO

        # Google's Soccer paper
        # a) scoring a goal --> +1
        # b) opponent half segmented in 10 slices. First time a slice is reached with ball, get +0.1 (10*0.1 == max 1.0)
        # if goal is scored from far away, all missed reward from b) is also rewarded
        # 'Gamma' value during training is used to reward the shortest path to maximum reward

        ownRobotPos = self.ws.getRobotPosition()
        ballPos = self.ws.getBallPosition()
        hasBall = self.ws.hasBall()

        oppGoalCenter = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALLINE_CENTER )
        oppGoalPostLeft = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALPOST_LEFT )
        oppGoalPostRight = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALPOST_RIGHT )

        # Compute delta ball distance (compared to previous step)
        distToBall = self._getDistRobotToBall()
        if not self.first_step:
            distToBallDelta = distToBall - self.prevDistToBall

        # Compute delta angle towards ball (compared to previous step)
        angleToBall = math.fabs( self._getAngleRobotToBall() )
        if not self.first_step:
            angleToBallDelta = angleToBall - self.prevAngleToBall

        # Compute delta angle towards goal (compared to previous step)
        angleToGoal = math.fabs( self._getAngleRobotToGoal() )
        if not self.first_step:
            angleToGoalDelta = angleToGoal - self.prevAngleToGoal

        # Compute if ball was grabbed
        if hasBall and not self.prevHasBall:
            ballGrabbed = True
            self.had_ball = True
        else:
            ballGrabbed = False

        # Compute delta distance of ball to goal (compared to previous step)
        distBallToGoal = (Vec2d(oppGoalCenter.x, oppGoalCenter.y) - ballPos).size()
        if not self.first_step:
            distBallToGoalDelta = distBallToGoal - self.prevDistBallToGoal

        # Store for next step
        self.prevDistToBall = distToBall
        self.prevAngleToBall = angleToBall
        self.prevAngleToGoal = angleToGoal
        self.prevHasBall = hasBall
        self.prevDistBallToGoal = distBallToGoal
            

        # Compute reward
        reward = 0.0

        if not self.first_step:
            # positive reward for decreasing distance to ball
            if not hasBall:
                reward += -distToBallDelta

            # positive reward for decreasing angle to goal if robot has ball
            # if has ball, positive reward for facing goal
            # otherwise, positive reward for facing ball
            if hasBall:
                reward += -angleToGoalDelta
            else:
                reward += -angleToBallDelta

            # If scored a goal, reward +20
            # If ball past the opponent line, reward -20
            if ballPos.y > oppGoalCenter.y:
                if oppGoalPostLeft.x < ballPos.x and ballPos.x < oppGoalPostRight.x:
                    reward += 20.0
                else:
                    reward -= 20.0

            # Negative reward for actions that FAIL
            action_result_value = self.rtdbStore.get(self.robotId, "ACTION_RESULT", timeout=None).value[0]
            if action_result_value == sharedTypes.actionResultTypeEnum["FAILED"].value:
                reward -= 500.0

            if ballGrabbed:
                reward += 10.0

            # Positive reward of ball moving toward goal after having the ball
            if self.had_ball:
                reward += -distBallToGoalDelta


        self.first_step = False
        return reward



    def _observe(self):

        ownRobotPos = self.ws.getRobotPosition()
        ballPos = self.ws.getBallPosition()
        hasBall = self.ws.hasBall()
        distRobotToBall = self._getDistRobotToBall()
        angleRobotToBall = self._getAngleRobotToBall()
        distRobotToGoal = self._getDistRobotToGoal()
        angleRobotToGoal = self._getAngleRobotToGoal()
        openingAngleGoal = self._getOpeningAngleGoal()
        
        return [ownRobotPos.x, ownRobotPos.y, ownRobotPos.Rz, ballPos.x, ballPos.y, hasBall, distRobotToBall, angleRobotToBall, distRobotToGoal, angleRobotToGoal, openingAngleGoal]

    def _stop_episode(self):

        result = False

        # stop when robot close to ball
        #distToBall = self._getDistRobotToBall()
        #if distToBall < 0.7:
        #    result = True

        # stop when ball crosses the opponent line
        oppGoalCenter = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALLINE_CENTER )
        if self.ws.getBallPosition().y > oppGoalCenter.y:
            result = True

        # or when an invalid action is chosen
        action_result_value = self.rtdbStore.get(self.robotId, "ACTION_RESULT", timeout=None).value[0]
        if action_result_value == sharedTypes.actionResultTypeEnum["FAILED"].value:
            result = True

        return result

    def _getDistRobotToBall(self):
        ballPos = self.ws.getBallPosition()
        ownRobotPos = self.ws.getRobotPosition()
        return (ownRobotPos.xy() - ballPos).size()

    def _getAngleRobotToBall(self):

        # Angle between robot.Rz and ball

        ownRobotPos = self.ws.getRobotPosition()
        ballPos = self.ws.getBallPosition()

        angleFacingBall = angle_between_two_points_0_2pi(ownRobotPos.x, ownRobotPos.y, ballPos.x, ballPos.y);

        return project_angle_mpi_pi( ownRobotPos.Rz - angleFacingBall )

    def _getDistRobotToGoal(self):
        oppGoalCenter = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALLINE_CENTER )
        ownRobotPos = self.ws.getRobotPosition()
        return ( ownRobotPos.xy() - Vec2d( oppGoalCenter.x, oppGoalCenter.y ) ).size()

    def _getAngleRobotToGoal(self):

        # Angle between robot.Rz and goalcenter

        ownRobotPos = self.ws.getRobotPosition()
        oppGoalCenter = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALLINE_CENTER )

        angleFacingGoal = angle_between_two_points_0_2pi(ownRobotPos.x, ownRobotPos.y, oppGoalCenter.x, oppGoalCenter.y);

        return project_angle_mpi_pi( ownRobotPos.Rz - angleFacingGoal )

    def _getOpeningAngleGoal(self):
        # Find two points for the angle
        # if goalie: TODO
        #   if goalie.x < 0:
        #     goalie <--> goalpost.right
        #   else:
        #     goalpost.left <--> goalie
        # else:
        #   goalpost.left <--> goalpost.right
        #
        # Transform both points to RCS
        # This puts the robot position as the origin (0, 0), so both points can be seen as vectors
        # Then we can compute the angle between both points

        ownRobotPos = self.ws.getRobotPosition()

        oppGoalPostLeft = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALPOST_LEFT )
        oppGoalPostRight = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALPOST_RIGHT )

        oppGoalPostLeftRCS = RobotPose(oppGoalPostLeft.x, oppGoalPostLeft.y, 0.0)
        oppGoalPostLeftRCS.transform_fcs2rcs(ownRobotPos)

        oppGoalPostRightRCS = RobotPose(oppGoalPostRight.x, oppGoalPostRight.y, 0.0)
        oppGoalPostRightRCS.transform_fcs2rcs(ownRobotPos)

        return angle_between_two_points_0_2pi(oppGoalPostLeftRCS.x, oppGoalPostLeftRCS.y, oppGoalPostRightRCS.x, oppGoalPostRightRCS.y)
        
    def _pause_simulation(self):
        # Set simulation tick frequency to 0 (pause simulation)
        coachId = 0
        executionConfig = self.rtdbStore.get(coachId, "CONFIG_EXECUTION", timeout=None)
        executionConfig.value["frequency"] = 0
        self.rtdbStore.put(coachId, "CONFIG_EXECUTION", executionConfig.value)

