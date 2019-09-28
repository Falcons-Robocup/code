""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import roslib # only for easy path management
roslib.load_manifest('geometry')
roslib.load_manifest('common')
roslib.load_manifest('sharedTypes')
roslib.load_manifest('rtdb3')

import threading, time

from FalconsCoordinates import *
from FalconsCommon import *
import FalconsEnv
from sharedTypes import * # generated enums
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH


class WorldState():

    def __init__(self):
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, True) # read mode
        self.rtdb2Store.refresh_rtdb_instances()
        self.robotId = FalconsEnv.get_robot_num()
        self.robotStatus = {}
        self.robotPosition = {}
        self.robotVelocity = {}
        self.robotHasBall = {}
        self.frequency = 20
        self.stopFlag = False
        self.updateThread = None
        
    def startMonitoring(self):
        self.updateThread = threading.Thread(target=self.updateLoop)
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
            v = self.rtdb2Store.get(robotId, "ROBOT_STATE")
            if v == None:
                # clear
                self.robotPosition[robotId] = None
            elif v.age() < 7.0:
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
        self.obstacles = []
        if v != None:
            for obst in v.value:
                self.obstacles.append(Vec2d(*obst[0]))

    def getRobotPosition(self, robotId):
        return self.robotPosition[robotId]

    def getRobotVelocity(self, robotId):
        return self.robotVelocity[robotId]

    def activeRobots(self):
        result = []
        for robotId in range(1, MAX_ROBOTS):
            if self.robotStatus.has_key(robotId):
                if self.robotStatus[robotId] == robotStatusEnum.INPLAY:
                    result.append(robotId)
        return result

    def hasBall(self, robotId):
        if self.robotHasBall.has_key(robotId):
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
        return self.obstacles

