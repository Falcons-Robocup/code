""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analyze a worldModel data stream.
# Intended for extracting KPI's:
#  * per robot
#    * distance driven (info)
#    * localization jumps (warning)
#    * intercept success rate (info)
#  * team level
#    * pass success rate (info)
#    * goal attempt success rate (info)
#    * searchball time (info)
#    * friendly collisions (warning)
#
# An event handler should be supplied in principle, otherwise information is simply sent to stdout.
#
# See also workShop 2016 Kassel - this design is intended to be more robust and better re-usable.
# Adapters can be written:
#  * to feed json worldmodel streams
#  * to feed our own match bag
#
# Jan Feitsma, 2017-04-26



# system includes
#import time
from math import floor, fmod, sqrt, atan2
from collections import defaultdict

# development debug include
from FalconsTrace import trace

# coordinate transformation
from FalconsCoordinates import Position2D

# event classification (consistent with t_events.msg)
EVENT_TYPE_INFO    = 0
EVENT_TYPE_WARNING = 1
EVENT_TYPE_ERROR   = 2
EVENT_ENUM2STR     = ["INFO", "WARNING", "ERROR"]

# ball possession (consistent with BallPossession.msg)
BALL_POSSESSION_TYPE_INVALID    = 0
BALL_POSSESSION_TYPE_FIELD      = 1
BALL_POSSESSION_TYPE_TEAMMEMBER = 2

# settings
ROBOT_IDS = range(1,7)
MIN_SPEED = 0.1 # to detect if robot is driving based on reported speed
FACING_ANGLE_TOLERANCE = 0.3
PASS_TIMEOUT = 5.0
LOC_GLITCH_DT = 0.3
LOC_GLITCH_TOL_XY = 1.0

# utilities
from utils import *
#TODO from cooldown import cooldown


# default event handler
def defaultEventHandler(eventString, eventType=EVENT_TYPE_INFO, robotNumber=0, timeStamp=None):
    if timeStamp == None:
        timeStamp = getTimeNow()
    s = time2str(timeStamp) + " - r" + str(robotNumber) + " - " + EVENT_ENUM2STR[eventType] + ": " + eventString
    trace("%s", s)
    print s


class AnalyzeWorldModel():

    def __init__(self, eventHandler=defaultEventHandler):
        # store eventhandler
        self.eventHandler = eventHandler
        # basic administration
        self.currPacket = None
        self.prevPacket = None
        self.timeStamp = 0
        self.timeStampStart = None
        # which robots were active at any moment in time
        self.activeRobots = {}
        # KPI searchBall: time wasted on searching for the ball
        self.timeStampBallLost = 0
        self.durationBallLost = 0
        # KPI distanceDriven: drive distance in meters (trying to not be sensitive to worldModel noise)
        self.distanceDriven = defaultdict(lambda: 0.0)
        # KPI localizationGlitches: 
        self.numLocGlitches = defaultdict(lambda: 0)
        # KPI passSuccessRate
        self.passAttemptActive = False
        self.numPassAttempts = 0
        self.numPassSuccess = 0
        # KPI shootOnGoalAttempts
        self.numShootOnGoalAttempts = 0
        
    def getTime(self):
        return self.timeStamp
        
    def handleEvent(self, eventMessage, eventType=EVENT_TYPE_INFO, robotId=0):
        # call the callback
        self.eventHandler(eventMessage, eventType, robotId, self.getTime())
        
    def feed(self, packet, timeStamp):
        trace("packet=%s", packet)
        self.prevPacket = self.currPacket
        self.prevTimeStamp = self.timeStamp
        self.currPacket = packet
        self.timeStamp = timeStamp
        if self.timeStampStart == None: 
            self.timeStampStart = timeStamp
        
    def display(self):
        elapsed = self.timeStamp - self.timeStampStart
        s = "analyzing team and robot performance:"
        self.handleEvent(s)
        s = "elapsed: %02dm %02ds" % (floor(elapsed / 60), fmod(elapsed, 60))
        self.handleEvent(s)
        # team KPI's
        s = "kpiSearchBall           =  %3d%%" % (int(round(100.0 * self.durationBallLost / elapsed)))
        self.handleEvent(s)
        s = "kpiShootOnGoalAttempts  =  %3d" % (self.numShootOnGoalAttempts)
        self.handleEvent(s)
        s = "kpiNumPassAttempts      =  %3d" % (self.numPassAttempts)
        self.handleEvent(s)
        s = "kpiNumPassSuccess       =  %3d" % (self.numPassSuccess)
        self.handleEvent(s)
        if self.numPassAttempts:
            s = "kpiPassSuccessRate      =  %3d%%" % (int(round(100.0 * self.numPassSuccess / self.numPassAttempts)))
        else:
            s = "kpiPassSuccessRate      =  n/a" # cannot divide by zero
        self.handleEvent(s)
        s = "kpiTotalDistanceDriven  = %6.1fm" % (sum([self.distanceDriven[irobot] for irobot in self.activeRobots.keys()]))
        self.handleEvent(s)
        # robot KPI's, grouped
        for irobot in self.activeRobots.keys():
            s = "kpiDistanceDriven(r%d)   = %6.1fm" % (irobot, self.distanceDriven[irobot])
            self.handleEvent(s, robotId=irobot)
        for irobot in self.activeRobots.keys():
            s = "kpiNumLocGlitches(r%d)   =  %3d" % (irobot, self.numLocGlitches[irobot])
            self.handleEvent(s, robotId=irobot)
        
    # several small helper functions
    
    def isDriving(self, robotId):
        currSpeed = max(abs(self.currPacket.robots[robotId].vx), abs(self.currPacket.robots[robotId].vy))
        return currSpeed > MIN_SPEED

    def gainedBallPossession(self):
        return self.currPacket.ballPossession.type == BALL_POSSESSION_TYPE_TEAMMEMBER and self.prevPacket.ballPossession.type != BALL_POSSESSION_TYPE_TEAMMEMBER

    def lostBallPossession(self):
        return self.currPacket.ballPossession.type != BALL_POSSESSION_TYPE_TEAMMEMBER and self.prevPacket.ballPossession.type == BALL_POSSESSION_TYPE_TEAMMEMBER

    def getPosition(self, robotId, prev=False):
        if prev:
            packet = self.prevPacket
        else:
            packet = self.currPacket
        return Position2D(packet.robots[robotId].x, packet.robots[robotId].y, packet.robots[robotId].phi)

    def calculateDistance(self, object1, object2):
        return sqrt((object1.x - object2.x) ** 2 + (object1.y - object2.y) ** 2)
    
    def robotFacesAnyFriend(self, robotId):
        """
        Return robot ID of any robot we are facing; return None if not facing any robot.
        """
        ourPos = self.getPosition(robotId)
        closestDist = 999
        result = None
        # check every friend
        for other in ROBOT_IDS:
            if self.currPacket.active[other] and other != robotId:
                otherFcs = self.getPosition(other)
                otherRcs = otherFcs.transform_fcs2rcs(ourPos)
                facingAngle = atan2(otherRcs.x, otherRcs.y)
                # check if we are facing this friend
                trace("facing angle of r%d wrt r%d : %6.2f", other, robotId, facingAngle)
                if abs(facingAngle) < FACING_ANGLE_TOLERANCE:
                    # there might be multiple friends within our facing cone, in which case we select closest friend
                    dist = self.calculateDistance(ourPos, otherFcs)
                    if dist < closestDist:
                        result = other
                        closestDist = dist
        return result

    def robotFacesGoal(self, robotId):
        """
        Determine if robot is facing opponent goal, for example attempting a shot.
        """
        ourPos = self.getPosition(robotId)
        goalFcs1 = Position2D(-1, 9, 0)
        goalFcs2 = Position2D( 1, 9, 0)
        goalRcs1 = goalFcs1.transform_fcs2rcs(ourPos)
        goalRcs2 = goalFcs2.transform_fcs2rcs(ourPos)
        facingAngle1 = atan2(goalRcs1.x, goalRcs1.y)
        facingAngle2 = atan2(goalRcs2.x, goalRcs2.y)
        trace("r%d phi=%.2f facingAngle1=%.2f facingAngle2=%.2f", robotId, ourPos.phi, facingAngle1, facingAngle2)
        return 0.0 > facingAngle1 and 0.0 < facingAngle2
        
    # main analysis sequence
                 
    def analyze(self):
        # check if we have a current and a previous packet
        if self.currPacket == None or self.prevPacket == None:
            return
        self.analyzeActiveRobots()
        self.analyzeSearchBall()
        self.analyzeDistanceDriven()
        self.analyzeBallPossessionAndShots()
        self.analyzeLocalizationGlitches()
        
    def analyzeActiveRobots(self):
        # maintain a list of robots which were active at any moment in time
        # used in display() to prevent useless output
        for irobot in ROBOT_IDS:
            if self.currPacket.active[irobot]:
                self.activeRobots[irobot] = True

    #@cooldown(1, getTime)
    # TODO fix cooldown decorator such that it can handle our simulated time in playback
    def reportLocalizationGlitch(self, glitchSize, robotId):
        self.handleEvent("localization glitch (%.1fm) detected" % (glitchSize), robotId=robotId)
        self.numLocGlitches[robotId] += 1
        
    def analyzeLocalizationGlitches(self):
        # cooldown to prevent reporting a single fast A-B-A glitch twice
        # check each active robot
        for irobot in ROBOT_IDS:
            if self.currPacket.active[irobot] and self.prevPacket.active[irobot]:
                deltaTime = self.timeStamp - self.prevTimeStamp
                if deltaTime < LOC_GLITCH_DT:
                    deltaPos = self.getPosition(irobot) - self.getPosition(irobot, prev=True)
                    glitchSize = max(abs(deltaPos.x), abs(deltaPos.y))
                    if glitchSize > LOC_GLITCH_TOL_XY:
                        self.reportLocalizationGlitch(glitchSize, irobot)
                        # check if the glitch is due to a robot flip, see TRAC ticket 304
                        if ((self.getPosition(irobot).x - (-1*self.getPosition(irobot, prev=True).x) < LOC_GLITCH_TOL_XY) and (self.getPosition(irobot).y - (-1*self.getPosition(irobot, prev=True).y) < LOC_GLITCH_TOL_XY)): # position is inverted
                            self.handleEvent("localization glitch likely due to robot flip")                        
                    # ignore phi for now, not really a problem - glitches tend to correlate
                else:
                    trace("unexpected large gap in data stream")
                
    def analyzeSearchBall(self):
        # KPI searchBall: time wasted on searching for the ball
        nBallCurrent = len(self.currPacket.balls)
        nBallPrevious = len(self.prevPacket.balls)
        if nBallCurrent == 0 and nBallPrevious > 0:
            self.handleEvent("lost track of the ball")
            self.timeStampBallLost = self.timeStamp
        if nBallPrevious == 0 and nBallCurrent > 0 and self.timeStampBallLost > 0:
            self.handleEvent("found the ball")
            self.durationBallLost += (self.timeStamp - self.timeStampBallLost)
            
    def analyzeDistanceDriven(self):
        # KPI distanceDriven: distance driven
        # do not accumulate fake distance while robot is standing still (noise)
        for irobot in ROBOT_IDS:
            if self.isDriving(irobot):
                dist = self.calculateDistance(self.currPacket.robots[irobot], self.prevPacket.robots[irobot])
                self.distanceDriven[irobot] += dist
                trace("r%d currPos=(%6.2f,%6.2f) dist=%6.2f", irobot, self.currPacket.robots[irobot].x, self.currPacket.robots[irobot].y, dist)

    def analyzeBallPossessionAndShots(self):
        # changes in ball possession can signal several things
        # we use such events to identify KPI passSuccessRate and shootAttempts
        
        # first some internal helper functions
        def startPassAttempt():
            # check if we are attempting a pass: we are facing a friendly robot
            robotFrom = int(self.prevPacket.ballPossession.robotID)
            robotTo = self.robotFacesAnyFriend(robotFrom)
            if robotTo != None:
                # we are indeed attempting a pass: generate event and register this for further analysis
                self.handleEvent("attempting a pass from r" + str(robotFrom) + " to r" + str(robotTo))
                self.passAttemptTimestamp = self.timeStamp
                self.passAttemptTarget = robotTo
                self.passAttemptActive = True
                self.numPassAttempts += 1
                return True
            self.passAttemptActive = False
            return False
        def endPassAttempt(robot):
            # check timeout
            if self.passAttemptActive:
                if self.passAttemptTimestamp - self.timeStamp > PASS_TIMEOUT:
                    self.handleEvent("pass timeout - failed")
                    self.passAttemptActive = False
            # check success 
            if self.passAttemptActive and robot != None:
                if robot == self.passAttemptTarget:
                    self.handleEvent("pass to r%d successful" % robot)
                    self.numPassSuccess += 1
                else:
                    self.handleEvent("pass failed - other robot got the ball")
        def startShootOnGoalAttempt():
            # check if we are attempting a shot on goal: we are facing opponent goal 
            # note: we rely precisely on accuracy of robot angle
            # TODO: do something with ball vector? time sensitive, complex ...
            robot = int(self.prevPacket.ballPossession.robotID)
            if self.robotFacesGoal(robot):
                # generate event and register this for further analysis
                # TODO: do something with ball speed because now we will get a few false positives due to bumping / scrums
                self.numShootOnGoalAttempts += 1
                self.handleEvent("attempting a shot on goal from r" + str(robot))

        # main analysis logic
        if self.lostBallPossession():
            # based on team geometry, determine what the intent is
            if not startPassAttempt():
                startShootOnGoalAttempt()
        elif self.gainedBallPossession():
            robotWithBall = int(self.currPacket.ballPossession.robotID)
            # if we were attempting a pass attempt recently, then success
            endPassAttempt(robotWithBall)
        else:
            # general timeouts to clear the pass/shot attempts
            endPassAttempt(None)
            
        # TODO determine the result of shoot on goal attempt: either successful, or blocked, or off target
        # this is a bit complex, since we do not have refbox feedback here...
        # ideally we would know the ball location and speed accurately at all times, 
        # then we could check if it ended up in the goal, or bounce
        # but due to vision and worldModel limitations this is not realistic
        # maybe solve in team.py with refbox feedback, triggering on the start attempt events generated here?


