# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0

import threading, time

import falconspy
from FalconsCoordinates import *
from FalconsCommon import *
import FalconsEnv
from sharedTypes import * # generated enums
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH


class WorldState():

    def __init__(self, robotId):
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, True) # read mode
        self.rtdb2Store.refresh_rtdb_instances()
        self.robotId = robotId
        self.robotStatus = {}
        self.robotPosition = {}
        self.robotVelocity = {}
        self.robotHasBall = {}
        self.frequency = 20
        self.stopFlag = False
        self.updateThread = None
        self.ball = None

    def startMonitoring(self):
        self.updateThread = threading.Thread(target=self.updateLoop)
        # daemon=True means thread is killed during shutdown of main thread, and not block main thread from shutdown
        self.updateThread.setDaemon(True)
        self.updateThread.start()

    def stopMonitoring(self):
        self.stopFlag = True

    def updateLoop(self):
        dt = 1.0 / self.frequency
        while self.stopFlag != True:
            time.sleep(dt)
            self.update()

    def update(self):
        """
        Retrieve all data from RTDB and store internally.
        """
        # ROBOT_STATE for all robots
        for robotId in range(1, MAX_ROBOTS):
            v = self.rtdb2Store.get(robotId, "ROBOT_STATE", timeout=7.0)
            if v == None:
                # clear
                self.robotPosition[robotId] = None
            else:
                # unpack struct
                status = robotStatusEnum(v.value[0])
                #timestamp = v.value[1] # ignore
                position = v.value[2]
                velocity = v.value[3]
                hasBall = v.value[4]
                #ballAcquired = v.value[5] # ignore
                # convert to better data types
                self.robotStatus[robotId] = status
                self.robotPosition[robotId] = RobotPose(*position)
                self.robotVelocity[robotId] = RobotVelocity(*velocity)
                self.robotHasBall[robotId] = hasBall
        # ball
        v = self.rtdb2Store.get(self.robotId, "BALLS")
        if v != None:
            if len(v.value):
                self.ball = v.value[0] # ignore any other ball, we expect worldModel to put best ball first
        # obstacles
        v = self.rtdb2Store.get(self.robotId, "OBSTACLES")
        self.obstaclePositions = []
        self.obstacleVelocities = []
        if v != None:
            for obst in v.value:
                self.obstaclePositions.append(Vec2d(*obst[0]))
                self.obstacleVelocities.append(Vec2d(*obst[1]))

    def getRobotPosition(self, robotId=None):
        if robotId == None:
            robotId = self.robotId
        return self.robotPosition[robotId]

    def getRobotVelocity(self, robotId=None):
        if robotId == None:
            robotId = self.robotId
        return self.robotVelocity[robotId]

    def activeRobots(self):
        result = []
        for robotId in range(1, MAX_ROBOTS):
            if robotId in self.robotStatus:
                if self.robotStatus[robotId] == robotStatusEnum.INPLAY:
                    result.append(robotId)
        return result

    def hasBall(self, robotId=None):
        if robotId == None:
            robotId = self.robotId
        if robotId in self.robotHasBall:
            return self.robotHasBall[robotId]
        return False

    def getBallPosition(self):
        if self.ball == None:
            return None
        return Vec3d(*self.ball[0])

    def getBallVelocity(self):
        if self.ball == None:
            return None
        return Vec3d(*self.ball[1])

    def ballPossession(self):
        if self.ball == None:
            return None
        return ballPossessionTypeEnum(self.ball[3][0])

    def getObstacles(self):
        return self.obstaclePositions

    def getObstacleVelocities(self):
        return self.obstacleVelocities

    def getRobotId(self):
        return self.robotId

    def ballDistance(self):
        r = self.getRobotPosition()
        b = self.getBallPosition()
        if b == None:
            return None
        return (r.xy() - Vec2d(b.x, b.y)).size()

