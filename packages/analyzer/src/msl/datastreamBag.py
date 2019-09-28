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
import rosbag
import traceback
import sys
import math

# package includes
from gamestate import gameState, worldState, refboxState, robotState
from datastreamMSL import dataStreamMSL, dataStream
from utils import project_angle_0_2pi
import events


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time




def fixPhi(phi):
    # Falcons FCS (field coordinate system) is defined with phi=0 coinciding with X axis
    # in other words, if a robot seen from goalie is taking a kickoff from left of the ball (-1,0,0) it has phi=0, facing right side
    # however, MSL logging requires phi=0 facing in forward playing direction, along the Y axis
    # so we need to correct 0.5*pi
    return project_angle_0_2pi(phi - 0.5*math.pi)


class dataStreamBag(dataStreamMSL):
    """
    Store the (partial) state of MSL match with timestamps based on Falcons .bag file.
    Used by analyzer to inspect the state on some fixed frequency.
    """

    def __init__(self):
        dataStreamMSL.__init__(self)
        self.teamNames = ["Falcons", "unknown"]

    def load(self, bagfile):
        """
        Load .bag file and store data streams.
        """
        events.info("loading {0} ...".format(bagfile), level=0)
        self.fileName = bagfile
        # guess opponent name from filename
        try:
            self.teamNames[1] = bagfile.split('_')[-1].split('.')[0]
        except:
            pass
        # load bag
        self._loadBag()
        # extract worldModel stream
        self.streams = {}
        self.streams[0] = self._loadBagWorldModel()
        # refbox
        self.streams['refbox'] = self._loadBagRefbox()

    # internal functions below

    def _loadBag(self):
        self._bagData = []
        bag = rosbag.Bag(self.fileName)
        for topic, msg, t in bag.read_messages():
            t1 = t.to_time()
            # Leipzig software had 2 coach topics
            # wmV2 uses /teamA/g_worldmodel_team
            if topic in ["/teamA/g_diag_refbox", "/teamA/g_worldmodel", "/teamA/g_worldmodel_team"]:
                self._bagData.append((t1, topic, msg))
 
    def _loadBagRefbox(self):
        events.info("parsing refbox stream ...", level=1)
        trace("parsing refbox stream")
        result = dataStream(refboxState)
        for e in self._bagData:
            if "refbox" in e[1]:
                m = e[2]
                # good line, convert columns
                r = refboxState()
                r.timeStamp = e[0]
                r.phase = "unused"
                r.lastSignal = self._translateSignal(m.refboxCmd)
                r.active = (r.lastSignal == "START")
                trace('t=%.3f signal=%s active=%d', r.timeStamp, r.lastSignal, r.active)
                # store - make sure each signal is stored (note that a REPAIR is sent right before START)
                # by messing a little bit with time stamps ...
                t = r.timeStamp
                while result.has_key(t):
                    t += 0.002
                r.timeStamp = t
                result.set(r, t)
        trace("finished parsing refbox stream")
        trace("datastream result: %s", result)
        events.info("datastream result: %s" % (result), level=1)
        return result
        
    def _loadBagWorldModel(self):
        # work through the data
        events.info("parsing .bag worldModel stream ...", level=1)
        trace("parsing .bag worldModel stream")
        result = dataStream(worldState)
        for e in self._bagData:
            timeStamp = e[0]
            msg = e[2]
            trace("t=%.3f msg=%s", timeStamp, msg)
            try:
                w = worldState()
                w.timeStamp = timeStamp # TODO warn for out-of-order entries? see for example Water data
                if "/teamA/g_worldmodel" == e[1]:
                    # wmV1
                    if len(msg.friends):
                        for r in msg.friends:
                            robotIdx = r.id
                            pose = (r.x, r.y, fixPhi(r.phi)) # correct Falcons FCS rotation offset
                            vel = (r.vx, r.vy, r.vphi)
                            if "None" in str(vel):
                                # initialize to zeros
                                vel = (0, 0, 0)
                            if not "None" in str(pose):
                                w.robots[robotIdx] = robotState(pose, vel)
                            # TODO: intention, targetPose
                    if len(msg.ballpos):
                        # select first ball in list
                        # TODO warn in case of multiple balls?
                        # TODO use confidence instead of assuming they are sorted descendingly?
                        ball = msg.ballpos[0]
                        if not "None" in str(ball):
                            w.ball = (ball.x, ball.y, ball.z)
                        # ignore ball speed for now
                    # ball possession
                    # before this commit (so wmV1), TEAM ball possession had enum value 3
                    # http://git.falcons-robocup.nl/falcons/code/commit/c783929459ed68c08064c6eb5149ec531ecf4022
                    if msg.ballPossession.type == 3:
                        w.robots[msg.ballPossession.robotID].ballEngaged = True
                if "/teamA/g_worldmodel_team" == e[1]:
                    # wmV2
                    for robotIdx in range(1, 7):
                        r = msg.robots[robotIdx]
                        if msg.active[robotIdx]:
                            pose = (r.x, r.y, fixPhi(r.phi)) # correct Falcons FCS rotation offset
                            vel = (r.vx, r.vy, r.vphi)
                            if "None" in str(vel):
                                # initialize to zeros
                                vel = (0, 0, 0)
                            if not "None" in str(pose):
                                w.robots[robotIdx] = robotState(pose, vel)
                    # TODO: intention, targetPose
                    if len(msg.balls):
                        # select first ball in list
                        # TODO warn in case of multiple balls?
                        # TODO use confidence instead of assuming they are sorted descendingly?
                        ball = msg.balls[0]
                        if not "None" in str(ball):
                            w.ball = (ball.x, ball.y, ball.z)
                        # ignore ball speed for now
                    # ball possession
                    # after this commit (so wmV2), TEAM ball possession has enum value 2
                    # http://git.falcons-robocup.nl/falcons/code/commit/c783929459ed68c08064c6eb5149ec531ecf4022
                    if msg.ballPossession.type == 2:
                        # somehow robot may not be active (yet?)
                        if w.robots.has_key(msg.ballPossession.robotID):
                            w.robots[msg.ballPossession.robotID].ballEngaged = True
            except:
                print msg
                traceback.print_exc(file=sys.stdout)
                raise
            # feed
            w.valid = True
            result.set(w, timeStamp)
        trace("finished parsing bag wm stream")
        trace("datastream result: %s", result)
        events.info("datastream result: %s" % (result), level=1)
        return result

    def _translateSignal(self, refboxCmd):
        # remap to the strings used in .msl logging
        # e.g. FREEKICK_OPP => 
        color = None
        if "_OWN" in refboxCmd:
            color = "CYAN"
            refboxCmd = refboxCmd.replace("_OWN", "")
        if "_OPP" in refboxCmd:
            color = "MAGENTA"
            refboxCmd = refboxCmd.replace("_OPP", "")
        mapping = {}
        mapping["START"] = "START"
        mapping["STOP"] = "STOP"
        mapping["FIRST_HALF"] = "1st half"
        mapping["SECOND_HALF"] = "2nd half"
        mapping["HALF_TIME"] = "Halftime"
        mapping["END_GAME"] = "End Game"
        mapping["FREEKICK"] = "Freekick"
        mapping["THROWIN"] = "Throw In"
        mapping["CORNER"] = "Corner"
        mapping["KICKOFF"] = "Kickoff"
        mapping["GOALKICK"] = "Goalkick"
        mapping["DROPPED_BALL"] = "Drop Ball"
        mapping["GOAL"] = "Goal+"
        mapping["SUBGOAL"] = "Goal-"
        mapping["PENALTY"] = "Penalty Kick"
        result = mapping[refboxCmd]
        if color != None:
            result = color + " " + result
        return result
        
