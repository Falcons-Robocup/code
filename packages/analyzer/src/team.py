""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Team class for analyzerRealtime (on coach): 
# * a few topic listeners
# * control all robot instances
# * determine averaged worldModel
# * check for inconsistencies
#
# Jan Feitsma, 2017-04-25



import sys,os
import traceback
from copy import copy
import roslib
import rospy
roslib.load_manifest('analyzer') 
from rosMsgs.msg import *
from FalconsTrace import trace
import time
from collections import defaultdict

# analyzer imports
from settings import *
from utils import getTimeNow, time2str
from robot import AnalyzeRobot
from analyzeWorldModel import AnalyzeWorldModel


class AnalyzeTeam():
    """
    This class performs team-level checks.
    For instance, it monitors which robots are online and will generate an error when a robot is disconnected.
    The errors are written in visualizer as big red letters.
    """
    def __init__(self, publish=True):
        # other
        self.lastRefboxCmd = None
        self.glitchStore = {}
        # construct analyzer with our own event handler
        self.wmAnalyzer = AnalyzeWorldModel(self.eventWrapper)
        self.wmAnalyzerDisplayed = {}
        self.doAnalyzeWm = True
        # initialize match state
        self.matchState = t_ana_matchstate()
        self.matchState.startTime = time.time()
        self.matchState.state = ""
        # output topics
        self.publish = publish
        if self.publish:
            self.pub_team_event = rospy.Publisher(TEAM_PREFIX + 'g_ana_event', t_event, queue_size = 30)
            # TODO replace online with comm
            self.pub_team_online = rospy.Publisher(TEAM_PREFIX + 'g_ana_online', t_ana_online, queue_size = 30)
            self.pub_team_matchstate = rospy.Publisher(TEAM_PREFIX + 'g_ana_matchstate', t_ana_matchstate, queue_size = 30)
            self.pub_team_worldmodel = rospy.Publisher(TEAM_PREFIX + 'g_worldmodel_team', t_worldmodel_team, queue_size = 30)
        # input topics
        self.subscribers = []
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'g_diag_refbox', t_diag_refbox, self.cbRefbox))
        # setup the robots
        self.robots = {}
        for robotnum in range(1,7):
            self.robots[robotnum] = AnalyzeRobot(self, robotnum, publish)
            
    def cbRefbox(self, msg):
        # NOTE: we assume this callback is triggered only on event!
        # so if we would start repeating the refbox on the on-coach topic, here the goal administration would get messed up..
        trace("cbRefbox: " + msg.refboxCmd)
        self.eventWrapper("refbox command: " + msg.refboxCmd, t_event.TYPE_INFO)
        self.lastRefboxCmd = (msg.refboxCmd, time.time())
        # update match state administration
        if "KICKOFF" in msg.refboxCmd:
            if self.matchState.state == "":
                self.matchState.state = "firstHalf"
            if self.matchState.state == "halfTime":
                self.matchState.state = "secondHalf"
        if "GOAL_OWN" == msg.refboxCmd:
            self.matchState.goalsOwn += 1
        if "GOAL_OPP" == msg.refboxCmd:
            self.matchState.goalsOpponent += 1
        if "HALF_TIME" == msg.refboxCmd:
            self.matchState.state = "halfTime"
        if "END_GAME" == msg.refboxCmd:
            self.matchState.state = "afterMatch"
                
    def getRobots(self, state=t_ana_online.ONLINE, exact=False):
        if exact:
            return [n for n in range(1,7) if self.robots[n].state == state]
        return [n for n in range(1,7) if self.robots[n].state >= state]
        
    def checkRobotState(self):
        # check if robots went offline 
        t = time.time()
        for robotnum in range(1,7):
            if t - self.robots[robotnum].lastAlive > ONLINE_TIMEOUT:
                self.robots[robotnum].state = t_ana_online.OFFLINE
            if t - self.robots[robotnum].lastWm > ONLINE_TIMEOUT:
                self.robots[robotnum].state = t_ana_online.ONLINE
        # fill message and publish
        msg = t_ana_online()
        msg.state = [0] # dummy at pos 0
        for robotnum in range(1,7):
            msg.state.append(self.robots[robotnum].state)
        if self.publish:
            self.pub_team_online.publish(msg)
        
    def eventWrapper(self, eventString, eventType, robotNumber=0, t=None):
        # TODO how to deal with duplicates here? timeout argument or string-based filtering? ...
        # TODO this is largely duplicate w.r.t. generateEvent above
        event = t_event()
        event.robotId = robotNumber
        event.fileName = 'analyzerRealtime.py'
        if t == None:
            t = getTimeNow()
        event.timeStamp = t
        event.eventType = eventType
        event.eventString = eventString
        self.publishEvent(event)
    
    def publishEvent(self, event):
        trace('publishEvent %s', event.eventString)
        if self.publish:
            self.pub_team_event.publish(event)
        # also print to stdout so we can watch it on coach, as some sort of error log
        print time2str(event.timeStamp) + " - r" + str(event.robotId) + " - " + event.eventString
        time.sleep(0.01) # workaround for superfast consecutive publishing, prevent ROS dropping messages
        
    def setMatchState(self):
        self.matchState.currentTime = time.time()
        #trace('setMatchState ' + str(self.matchState.currentTime))
        if self.lastRefboxCmd != None:
            self.matchState.lastRefboxCommand = self.lastRefboxCmd[0]
            self.matchState.lastRefboxAge = (self.matchState.currentTime - self.lastRefboxCmd[1])
        if self.publish:
            self.pub_team_matchstate.publish(self.matchState)
        
    def checkRefbox(self):
        #trace('checkRefbox')
        if self.lastRefboxCmd != None:
            elapsed = time.time() - self.lastRefboxCmd[1]
            trace('elapsed = %6.2f' % (elapsed))
            if elapsed > REFBOX_CHECK_TIME:
                ok = True
                for robotnum in self.getRobots(t_ana_online.INGAME):
                    if self.robots[robotnum].lastRefboxCmd != self.lastRefboxCmd[0]:
                        ok = False
                        eventString = 'refbox inconsistency (%s)' % (self.robots[robotnum].lastRefboxCmd)
                        self.eventWrapper(eventString, t_event.TYPE_WARNING, robotnum)
                trace('ok=' + str(ok))
                if ok:
                    # no need to check again until next refbox signal occurs
                    self.lastRefboxCmd = None
    
    def checkRoleAssignment(self):
        #trace('checkRoleAssignment')
        roles = {}
        # check if every role in array is unique
        onlineRobots = self.getRobots(t_ana_online.INGAME) 
        for robotnumA in onlineRobots:
            robotA = self.robots[robotnumA] 
            if robotA.role != None and robotA.role != "":
                if robotA.role !=  'R_robotStop': # ignore, all robots get this at the same time
                    # check if any of the other robots have this role
                    for robotnumB in onlineRobots: 
                       if robotnumB > robotnumA:
                           robotB = self.robots[robotnumB]                         
                           trace("compare roles " + robotA.role + " " + robotB.role)
                           if robotA.role == robotB.role:
                                # log error
                                eventString = "RoleAssignment inconsistency: robot%d and robot%d have role %s " % (robotnumA, robotnumB, robotA.role)
                                self.eventWrapper(eventString, t_event.TYPE_WARNING)
           
    # a glitch is an event which can be triggered very briefly
    # if it is triggered continuously for more than a brief moment, it is promoted to an event
    def handleGlitch(self, allowedTime, glitch):
        trace("detected possible glitch: " + str(glitch))
        # check if glitch is new (then just store) or existing (might need to generate event)
        if self.glitchStore.has_key(glitch):
            # send if the glitch was already stored a while ago
            if time.time() - self.glitchStore[glitch] > allowedTime:
                # it must really be a glitch, so generate an event
                trace("publishing event: " + str(glitch))
                self.eventWrapper(*glitch)
            # cleanup to reduce spam
            del self.glitchStore[glitch]
        else:
            # store glitch with its timestamp
            self.glitchStore[glitch] = time.time()
        # cleanup outdated items in glitch store
        for g in self.glitchStore.keys():
            if self.glitchStore[g] < time.time() - 2:
                self.resetGlitch(g)
        
    def resetGlitch(self, glitch):
        if self.glitchStore.has_key(glitch):
            del self.glitchStore[glitch]
            
    def ballPossessionString(self, bp):
        s = ''
        if bp.type == BallPossession.TYPE_FIELD:
            s = 'field'
        elif bp.type == BallPossession.TYPE_TEAMMEMBER:
            s = 'r' + str(int(bp.robotID))
        else:
            s = 'invalid'
        return s

    def averageObjects(self, objects, numActive, xyTol=3):
        # reduce list of near-duplicates to an averaged view
        #trace("inputN=%d objects=%s numActive=%d", len(objects), str(objects), numActive)
        result = []
        attrs = ['x', 'y', 'vx', 'vy'] # TODO dynamic introspection on objType? z ball
        nattr = len(attrs)
        tStart = time.time()
        while len(objects):
            # pick any object
            o = objects.pop()
            avg = copy(o)
            #trace("init avg=%s", str(avg))
            nMatch = 1
            seen = []
            # find matching ones
            for it in reversed(range(len(objects))):
                e = objects[it]
                d2 = ((e.x - o.x) ** 2 + (e.y - o.y) ** 2)
                # only if both distance is close and robotId does not match 
                if (d2 < xyTol * xyTol) and (e.id not in seen):
                    # match, accumulate, delete
                    nMatch = 1 + nMatch
                    seen.append(e.id)
                    for iattr in range(nattr):
                        setattr(avg, attrs[iattr], getattr(avg, attrs[iattr]) + getattr(e, attrs[iattr]))
                    del objects[it]
            # average
            for iattr in range(nattr):
                setattr(avg, attrs[iattr], getattr(avg, attrs[iattr]) / nMatch)
            # store in result, remove from list of objects
            #trace("glued %d objects into %s", nMatch, str(avg))
            # TODO set confidence based on contributing number of objects?
            # TODO do something special for objects which deviate from the average?
            # artificial ID counter, fits with the way visualizer handles id (reuse VTK actors with existing id)
            avg.id = len(result)
            result.append(avg)
        #trace("outputN=%d, resultStr=%s", len(result), str(result))
        trace("outputN=%d, elapsed=%.2fs", len(result), time.time() - tStart)
        return result
        
    def checkWorldModelConsistency(self):
        # for every participating robot: compare worldModel data, identify inconsistencies 
        # publish the average as 'team' worldModel, which then is used in visualizer and refbox feedback
        # allow inconsistencies only for a short while due to timing sensitivity (see 'glitch' functions)
        # initialize
        wmAverage = t_worldmodel_team()
        # online, fixed-length array, index is robotnum (so 0 unused)
        activeRobots = self.getRobots(t_ana_online.ACTIVE)
        inplayRobots = self.getRobots(t_ana_online.INPLAY)
        trace("inplayRobots=%s", str(inplayRobots))
        wmAverage.active = [r in inplayRobots for r in range(7)] # use INPLAY for analyzer consistency checks etc
        # simply copy robot position and speed, as reported by themselves, no need to check consistencies
        # fixed-length array, index is robotnum (so 0 unused)
        for robotnum in range(7):
            # TODO: only if inplay, otherwise we get false positive localization glitches at init time
            if robotnum in activeRobots:
                wmAverage.robots.append(self.robots[robotnum].posvel)
            else:
                wmAverage.robots.append(t_posvel())
        # inplay
        activeStringTeam = str(inplayRobots).translate(None, '[],')
        for robotnum in inplayRobots:
            activeStringRobot = self.robots[robotnum].wmTop.teamActivity[:-1] # ignore trailing space, TODO solve at producer
            glitch = ("inplay inconsistency (robot:[%s], team:[%s])" % (activeStringRobot, activeStringTeam), t_event.TYPE_WARNING, robotnum)
            if activeStringRobot != activeStringTeam:
                self.handleGlitch(GLITCH_ALLOWED_TIME, glitch)
            else:
                self.resetGlitch(glitch)
        # ball possession
        if len(inplayRobots):
            possessionCount = defaultdict(lambda: 0)
            for robotnum in inplayRobots:
                possessionCount[self.robots[robotnum].wmTop.ballPossession] += 1
            # find and copy best result
            m = 0
            for e in possessionCount.keys():
                if possessionCount[e] > m:
                    m = possessionCount[e]
                    wmAverage.ballPossession = e
            # find irregularities
            ballPossessionStringTeam = self.ballPossessionString(wmAverage.ballPossession)
            for robotnum in inplayRobots:
                ballPossessionStringRobot = self.ballPossessionString(self.robots[robotnum].wmTop.ballPossession)
                glitch = ("ball possession inconsistency (robot:%s, team:%s)" % (ballPossessionStringRobot, ballPossessionStringTeam), t_event.TYPE_WARNING, robotnum)
                if ballPossessionStringRobot != ballPossessionStringTeam:
                    self.handleGlitch(GLITCH_ALLOWED_TIME, glitch)
                else:
                    self.resetGlitch(glitch)
        # number of balls
        if 0 * len(inplayRobots): # JFEI disabled due to tuning differences (e.g. r1 is tuned much more aggressively, lower timeout)
            ballCount = [len(self.robots[robotnum].balls) for robotnum in inplayRobots]
            numBallsTeam = max(ballCount) # not necessarily max-occurrence...
            for robotnum in inplayRobots:
                numBallsRobot = len(self.robots[robotnum].balls)
                glitch = ("ball count inconsistency (robot:%d, team:%d)" % (numBallsRobot, numBallsTeam), t_event.TYPE_WARNING, robotnum)
                if numBallsRobot != numBallsTeam:
                    self.handleGlitch(GLITCH_ALLOWED_TIME, glitch)
                else:
                    self.resetGlitch(glitch)
        # ball(s) position&speed
        if len(inplayRobots):
            allBalls = sum([self.robots[robotnum].balls for robotnum in inplayRobots], [])
            wmAverage.balls = self.averageObjects(allBalls, len(inplayRobots))
        # TODO also send inconsistencies to be drawn as red or something?
        # obstacles
        if len(inplayRobots):
            allObstacles = sum([self.robots[robotnum].obstacles for robotnum in inplayRobots], [])
            wmAverage.obstacles = self.averageObjects(allObstacles, len(inplayRobots), xyTol=0.3)
        # store
        self.avgWm = wmAverage
        # publish team average on self.pub_team_worldmodel, for the following purposes:
        # * visualize it as team average (the topic is stored in bag and processed by visualizer as 'team' view)
        # * refbox feedback (via refboxRelay)
        if self.publish:
            self.pub_team_worldmodel.publish(wmAverage)
    
    def analyzeWorldModel(self):
        """
        Analyze the team-averaged worldModel data.
        Actually, dispatch the work to a separate module (which we might reuse on robot, or when analyzing other data streams from other teams, or ..).
        """
        self.wmAnalyzer.feed(self.avgWm, getTimeNow())
        self.wmAnalyzer.analyze()
        # if a playing half has ended, then display the KPI's
        # TODO: maybe also do this on-demand (dev testing) or periodically?
        if self.matchState.state in ["halfTime", "afterMatch"]:
            # only if not printed already...
            if not self.wmAnalyzerDisplayed.has_key(self.matchState.state):
                self.wmAnalyzer.display()
                self.wmAnalyzerDisplayed[self.matchState.state] = True
    
    def update(self):
        # tick
        self.setMatchState()
        self.checkRobotState()
        self.checkRefbox()
        self.checkRoleAssignment()
        self.checkWorldModelConsistency() # also performs worldModel averaging
        try:
            if self.doAnalyzeWm:
                self.analyzeWorldModel()
        except:
            errorStr = str(sys.exc_info()[0])
            errorStr += str(traceback.format_exc())
            self.eventWrapper(errorStr, t_event.TYPE_ERROR)
            self.doAnalyzeWm = False
        # poke robots
        for robotnum in self.getRobots():
            self.robots[robotnum].update()

