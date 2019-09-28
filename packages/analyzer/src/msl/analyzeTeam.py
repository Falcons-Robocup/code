""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 
# FALCONS // Jan Feitsma, August 2017



# python includes
from collections import defaultdict

# package includes
import events
from utils import Pose
import settings
from analyzeRobot import analyzeRobot


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


class analyzeTeam():
    def __init__(self, teamName, statistics):
        self.teamName = teamName
        self.robots = defaultdict(lambda : analyzeRobot())
        self.statistics = statistics
        self.statistics['teamName'] = teamName
        self.currentTimeStamp = None
        self.currentState = None
        self.previousTimeStamp = None
        self.previousState = None

    def update(self, state, timeStamp, active):
        """
        Analyze the team state at given timestamp.
        Called by analyzer on some fixed frequency.
        This function should not be called when the game is not active, because data can easily get tainted 
        (robots being taken inplay / outofplay, ...).
        """
        trace("update t=%.3f active=%d valid=%d", timeStamp, active, state.valid)
        # store for use in member functions
        self.currentTimeStamp = timeStamp
        self.currentState = state
        # do the work, but only if game is active
        if active:
            # check if team has data at all (for instance single MSL stream)
            if state.valid:
                # calculate data quality
                if not self.dataQualityOk():
                    self.statistics['dataQualityWarnings'] += 1
                    #events.warning("in-game data quality not OK for team %s" % (self.teamName), timeStamp=self.currentTimeStamp, timeout=2)
                else:
                    # only analyze 'reliable' (fresh) data
                    self.analyze()
        # update previous
        self.previousTimeStamp = timeStamp
        self.previousState = state

    def analyze(self):
        """
        Analyze current state, compare with previous.
        Assume data quality is OK.
        Update statistics where applicable.
        """
        # availability: number of participating robots
        # at every tick a maximum of 5 robots can contribute
        numRobots = self.numPlayingRobots()
        if numRobots > 5:
            events.warning("too many (%d) playing robots for team %s"
                           % (numRobots, self.teamName), timeStamp=self.currentTimeStamp, timeout=6000)
            numRobots = 5 # clip to prevent messing up availability calculation
        if numRobots < 3:
            events.warning("too few (%d) playing robots for team %s" % (numRobots, self.teamName), timeStamp=self.currentTimeStamp, timeout=6000)
        self.statistics.addAvailability(numRobots)
        # distance driven; teleports / vision glitches
        self.checkMovement()
        # ball location
        self.checkBallPosition()

    def dataQualityOk(self):
        dt = self.currentState.timeStamp - self.previousState.timeStamp
        if (dt >= settings.DATA_QUALITY_TIME_THRESHOLD):
            trace("t_curr=%.3f t_prev=%.3f dt=%.3f", self.currentState.timeStamp, self.previousState.timeStamp, dt)
        return dt < settings.DATA_QUALITY_TIME_THRESHOLD

    def numPlayingRobots(self):
        # trace state change 
        previousPlayingRobots = self.playingRobots(self.previousState)
        currentPlayingRobots = self.playingRobots(self.currentState)
        if previousPlayingRobots != currentPlayingRobots:
            events.info("change in playing robots for team %s: %s"
                        % (self.teamName, currentPlayingRobots), timeStamp=self.currentTimeStamp, timeout=6000, level=3)
        return len(self.playingRobots(self.currentState))
        
    def playingRobots(self, state):
        result = []
        for robotId in state.robots.keys():
            # it is not enough to simply check the presence of data
            # consider for instance a robot which is being serviced -- inplay/outofplay is not part of JSON protocol
            # see for instance TechUnited match against Water 20160701_095403: 1 robot is continuously generating data
            # furthermore it can happen that a robot is standing still on the field, generating data but not playing
            # so we should apply some extra criteria
            robot = self.robots[robotId]
            if robot != None:
                pose = robot.getPose()
                isPlaying = False
                if pose.insideKeeperArea():
                    isPlaying = True
                else:
                    # robot on field must have moved a bit recently
                    if pose.onField(dist=0.0):# and not robot.frozen(age=10.0):
                        isPlaying = True
                    # robot outside field (throwin, corner) is allowed to stand still longer
                    # but we only count between refbox start/stop anyway, so no need to check further
                if isPlaying:
                    result.append(robotId)
        return result        
        
    def checkMovement(self):
        """
        Check movement of a robot. If a robot appears to have teleported, then possibly it suffered 
        from a vision / compass issue.
        Otherwise accumulate distance driven and inspect speed.
        """
        for robotId in self.currentState.robots.keys():
            # construct if new
            self.robots[robotId].id = "%s/r%d" % (self.teamName, robotId)
            # inspect velocity as reported by the robot itself
            # note that this basically is unreliable data, due to for instance teams not filling it in, 
            # providing nonsense / poorly calibrated values, temporarily slipping wheels causing large spikes
            ownSpeed = self.currentState.robots[robotId].velocity.r()
            if ownSpeed > self.statistics['maxSpeedOwn']:
                trace("new maximum speed reported: %.1fm/s at t=%.3f", ownSpeed, self.currentState.timeStamp)
                self.statistics['maxSpeedOwn'] = ownSpeed
            # feed current pose to analyzeRobot, which can do things like teleportation detection / smoothening
            self.robots[robotId].feedPose(self.currentState.robots[robotId].pose, self.currentState.timeStamp)
            # calculate distance driven since last update
            tCurr = self.currentState.timeStamp
            tPrev = self.previousState.timeStamp
            currentPose = self.robots[robotId].getPose(tCurr)
            previousPose = self.robots[robotId].getPose(tPrev)
            assert(currentPose != None)
            if (previousPose == None):
                # only got 1 sample so far...
                continue
            distance = (currentPose - previousPose).r()
            # calculate speed
            # prevent dividing by small numbers, which blow up noise
            dt = tCurr - tPrev
            if dt < 0.12: 
                continue
            fitSpeed = distance / dt
            trace("team=%s robot=%d distance=%.1fm dt=%.2fs ownSpeed=%.1fm/s fitSpeed=%.1fm/s", 
                  self.teamName, robotId, distance, dt, ownSpeed, fitSpeed)
            # check for teleport
            if distance > settings.TELEPORT_DISTANCE:
                self.statistics['teleports'] += 1
                events.warning("robot %d of team %s teleported %.1fm"
                       % (robotId, self.teamName, distance), timeStamp=self.currentTimeStamp)
            else:
                # update statistics
                if (fitSpeed > 0.1 and fitSpeed < 6.0): # noise reduction and sanity check
                    self.statistics['distanceDriven'] += distance
                    self.statistics['maxSpeedFit'] = max(fitSpeed, self.statistics['maxSpeedFit'])
                     
    def checkBallPosition(self):
        """
        Check if team sees the ball. If so, put it in an area.
        The ball is either:
        * not seen         (ballLost)
        * at center area   (ballAtCenter)
        * at own half      (ballAtOwnHalf)
        * or opponent half (ballAtOpponentHalf)
        At each iteration, the applicable counter is increased and final statistics show percentages.
        """
        if self.currentState.ball == None:
            self.statistics['ballLost'] += 1
        else:
            # check how high robots can see the ball
            ballZ = self.currentState.ball[2]
            self.statistics['ballMaxZ'] = max(ballZ, self.statistics['ballMaxZ'])
            # check where the ball is on the field (offense / defense)
            # note: data of both teams is in FCS, so -Y is own half, +Y opponent half
            ballY = self.currentState.ball[1]
            if abs(ballY) < settings.BALL_CENTER_BAND:
                self.statistics['ballAtCenter'] += 1
            else:
                if ballY > 0:
                    self.statistics['ballAtOpponentHalf'] += 1
                if ballY < 0:
                    self.statistics['ballAtOwnHalf'] += 1
                    

