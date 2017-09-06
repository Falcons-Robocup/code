""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Robot class for analyzerRealtime (on coach): topic listeners, determine state of robot
#
# Jan Feitsma, 2017-04-25


import roslib
import rospy
roslib.load_manifest('analyzer') 
from rosMsgs.msg import *
from FalconsTrace import trace
import time

# analyzer imports
from settings import *
from utils import getTimeNow, time2str


class AnalyzeRobot():
    """
    This class performs robot-level checks.
    """
    def __init__(self, team, robotnum, publish=True):
        self.publish = publish
        self.team = team
        self.state = 0
        self.robotnum = robotnum
        self.lastAlive = 0
        self.lastWm = 0
        self.lastRefboxCmd = ""
        self.lastRefboxTime = 0
        self.seenEvents = {}
        self.packetHistory = {}
        self.lastDiskUsageWarn = 0
        self.role = None
        self.balls = []
        self.obstacles = []
        # output topics
        self.pub_comm_kpi = rospy.Publisher(TEAM_PREFIX + 'robot%d/g_ana_comm' % (robotnum), t_ana_comm, queue_size = 3)
        # input topics
        self.subscribers = []
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_health_fast' % (robotnum), t_diag_health_fast, self.cbHealthFast))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_health_mid' % (robotnum), t_diag_health_mid, self.cbHealthMid))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_health_slow' % (robotnum), t_diag_health_slow, self.cbHealthSlow))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_events' % (robotnum), t_diag_events, self.cbEvents))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_refbox' % (robotnum), t_diag_refbox, self.cbRefbox))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_teamplay' % (robotnum), t_diag_teamplay, self.cbTeamplay))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_wm_top' % (robotnum), t_diag_wm_top, self.cbWmTop))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_wm_loc' % (robotnum), t_diag_wm_loc, self.cbWmLoc))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_wm_ball' % (robotnum), t_diag_wm_ball, self.cbWmBall))
        self.subscribers.append(rospy.Subscriber(TEAM_PREFIX + 'robot%d/g_diag_wm_obstacles' % (robotnum), t_diag_wm_obstacles, self.cbWmObstacles))
            
    def cbHealthFast(self, msg):
        # ignore message, just store timestamp
        self.lastAlive = time.time()
        # go from offline to online?
        if self.state == t_ana_online.OFFLINE:
            self.state = t_ana_online.ONLINE

    def calculateCommKPI(self):
        # clear out old packets
        tThreshold = getTimeNow() - COMM_BUFFER_SIZE
        for k in self.packetHistory.keys():
            if self.packetHistory[k][1] < tThreshold:
                del self.packetHistory[k]
        # so now we have a buffer with all received events over the last X seconds
        commMsg = t_ana_comm()
        # need at least 2 packets
        if len(self.packetHistory) < 2:
            return
        # calculate packet frequency (should be equal to the frequency as set in robotControl, 
        # but might be higher when events are actually generated, because we want them to be sent immediately
        commMsg.frequency = len(self.packetHistory) / COMM_BUFFER_SIZE
        # calculate packet loss as fraction by comparing first and last index, counting gaps in between
        indexOldest = min([e[0].id for e in self.packetHistory.values()])
        indexNewest = max([e[0].id for e in self.packetHistory.values()])
        numExpectedPackets = indexNewest - indexOldest + 1
        numLostPackets = numExpectedPackets - len(self.packetHistory)
        commMsg.packetLoss = numLostPackets * 1.0 / numExpectedPackets
        # calculate latency as average timestamp difference
        timeDiffTotal = sum([e[1] - e[0].timeStamp for e in self.packetHistory.values()])
        commMsg.latency = timeDiffTotal / len(self.packetHistory)
        # publish
        if self.publish:
            self.pub_comm_kpi.publish(commMsg)
        
    def cbEvents(self, msg):
        #trace("callback triggered: sent=%16.6f elapsed=%16.6f", msg.timeStamp, getTimeNow() - msg.timeStamp)
        #trace(str(msg))
        # store packet for measuring packet loss and latency / time offset
        self.packetHistory[msg.id] = (msg, getTimeNow())
        # forward events, but filter duplicates
        for e in msg.events:
            if not self.seenEvents.has_key(e.eventId):
                self.team.publishEvent(e)
                self.seenEvents[e.eventId] = e
        
    def generateEvent(self, eventString, eventType):
        # TODO how to deal with duplicates here? timeout argument or string-based filtering? ...
        event = t_event()
        event.robotId = self.robotnum
        # note: eventId is not used in this context, only used for uniquely filtering events originating from robot
        event.fileName = 'analyzerRealtime.py'
        event.timeStamp = getTimeNow()
        event.eventType = eventType
        event.eventString = eventString
        self.team.publishEvent(event)
    
    def cbHealthMid(self, msg):
        pass
        
    def cbHealthSlow(self, msg):
        if int(msg.diskUsage) > DISK_USAGE_WARNING_THRESHOLD:
            # check last warning percentage (to avoid spamming)
            if int(msg.diskUsage) > self.lastDiskUsageWarn:
                eventString = "disk usage: " + str(msg.diskUsage) + "%"
                self.generateEvent(eventString, t_event.TYPE_WARNING)
                self.lastDiskUsageWarn = int(msg.diskUsage)

    def cbRefbox(self, msg):
        #trace(msg.refboxCmd)
        self.lastRefboxCmd = msg.refboxCmd
        self.lastRefboxTime = time.time()
        # update robot state
        self.state = t_ana_online.INGAME
        
    def cbTeamplay(self, msg):
        # store the role for team analysis
        trace("got role %s for robot %d", self.role, self.robotnum)
        self.role = msg.role
        
    def cbWmTop(self, msg):
        self.wmTop = msg
        self.lastWm = time.time()
        # TODO we might need a state machine...
        if self.state in [t_ana_online.INPLAY, t_ana_online.ACTIVE]:
            if msg.inplay:
                self.state = t_ana_online.INPLAY
            if msg.inplay == False:
                self.state = t_ana_online.ACTIVE
        if self.state == t_ana_online.INGAME and not msg.inplay:
            self.state = t_ana_online.ACTIVE
        
    def cbWmLoc(self, msg):
        self.posvel = msg.ownpos
        # state updates
        if (self.state == t_ana_online.ONLINE) and (self.posvel.x == 0.0) and (self.posvel.y == 0.0):
            self.state = t_ana_online.STARTING
        if (self.state == t_ana_online.STARTING) and ((self.posvel.x != 0.0) or (self.posvel.y != 0.0)):
            self.state = t_ana_online.ACTIVE
        if (self.state < t_ana_online.ACTIVE) and ((self.posvel.x != 0.0) or (self.posvel.y != 0.0)):
            self.state = t_ana_online.ACTIVE
        
    def cbWmBall(self, msg):
        self.balls = msg.balls
        # for averaging put robotnum in id; we do not care here about trackerID
        for b in self.balls:
            b.id = self.robotnum
    
    def cbWmObstacles(self, msg):
        # convert data type from t_obstacle to t_object
        self.obstacles = [] 
        for o1 in msg.obstacles:
            o2 = t_object()
            o2.id = self.robotnum # use robotID so we can do proper averaging; tracker ID is not relevant anyway
            o2.x = o1.x
            o2.y = o1.y
            o2.vx = o1.vx
            o2.vy = o1.vy
            self.obstacles.append(o2)
    
    def update(self):
        self.calculateCommKPI()

