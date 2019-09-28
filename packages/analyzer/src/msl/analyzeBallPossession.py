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
from collections import namedtuple, defaultdict

# package includes
from utils import Pose
from datastream import dataStream
import settings
import events


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


# if a player loses ball possession ('disengages' the ball), then one of the following holds:
# * normal situations
#    N1 player attempts a shot towards the goal
#    N2 player attempts a pass to teammate
#    N3 player performs a self-pass
#    N4 player loses the ball due to being bumped and/or poor ballHandling
# * exceptional situations:
#    E1 player shoots the ball away but not towards goal nor teammate
#    E2 player incorrectly claims ballPossession while ball is somewhere else (sensor / worldModel error)
#    E3 player incorrectly fails to claim ballPossession while he actually has the ball (sensor / worldModel error)
#
# characteristics of a shot on goal (N1):
# * player A loses ball possession while roughly aiming at the goal
# * no other players acquire ball possession for the next few seconds
#
# characteristics of a regular pass (N2):
# * player A loses ball possession facing player B
# * a short while later (<3 seconds) player B gains ball possession
# * no other players acquire ball possession in the meantime
#
# characteristics of a self-pass (N3):
# * player A loses ball possession
# * a short while later (<1 seconds) player A gains ball possession a small distance away (player is moving)
# * no other players acquire ball possession in the meantime
#  
# the normal situations N1, N2 and N3 (along with pass success rate) can be identified by inspecting geometry and timing of the disengage occurrence and the next ball engage occurrence
# * if the ball is engaged by the same player within 1 second, a self-pass is registered
# * otherwise, if the ball is engaged by a friendly player which the original player was (roughly) aiming at, a successful pass is registered (N2success)
# * otherwise, if after a few seconds the ball is not claimed by anyone:
#     if the ball was aimed roughly at the goal, a shot on goal is registered (N1)
#     if the ball was aimed at a friendly player, a failed pass attempt is registered (N2fail)
# the other situations are not identified at the moment, because these are much harder to determine


# auxiliary datatypes
ballPossessionChangeEvent = namedtuple("ballPossessionChangeEvent", "timeStamp engaged owner location aimTargets")
ballPossessionChangeEvent.__new__.__defaults__ = (None,) * len(ballPossessionChangeEvent._fields)
ballOwnerShip = namedtuple("ballOwnerShip", "team robot multiple")
ballOwnerShip.__new__.__defaults__ = (None,) * len(ballOwnerShip._fields)


class analyzeBallPossession():
    def __init__(self, statistics):
        self.statistics = statistics
        self.buffer = dataStream(ballPossessionChangeEvent)
        self.evaluated = defaultdict(lambda : False)
        self.location = None

    def update(self, teamStates, timeStamp):
        # check which of the robots currently think they engage the ball
        count = 0
        owner = None
        for teamIdx in [0, 1]:
            #trace("team=%d valid=%d nrobot=%d", teamIdx, teamStates[teamIdx].valid, len(teamStates[teamIdx].robots))
            if teamStates[teamIdx].valid:
                for (robotId, robot) in teamStates[teamIdx].robots.iteritems():
                    if robot.ballEngaged:
                        count += 1
                        if count == 1:
                            owner = ballOwnerShip(team=teamIdx, robot=robotId, multiple=False)
                            self.location = robot.pose
                        else:
                            owner = ballOwnerShip(team=teamIdx, robot=robotId, multiple=True) # scrum!
        # determine currentState
        currentEngaged = (owner != None)
        currentOwner = owner
        currentLocation = self.location
        # get previous state
        last = self.buffer.get(timeStamp)
        # register change?
        if (last.engaged, last.owner) != (currentEngaged, currentOwner):
            # update basic engage/disengage statistics
            aimTargets = []
            if currentEngaged:
                if currentOwner.multiple:
                    events.info("multiple robots are engaged in a scrum", level=3, timeStamp=timeStamp)
                    self.statistics.match['ballScrums'] += 1
                else:
                    teamName = self.statistics.teams[currentOwner.team]['teamName']
                    events.info("robot %d from team %s has engaged the ball" % (currentOwner.robot, teamName), level=3, timeStamp=timeStamp)
                    self.statistics.teams[currentOwner.team]['ballEngaged'] += 1
            else:
                events.info("ball has been disengaged", level=3, timeStamp=timeStamp)
                # extra info is required for determining pass intent: 
                # was robot aiming at friend while disengaging the ball?
                if last.engaged and teamStates[last.owner.team].valid:
                   for (robotId, robot) in teamStates[last.owner.team].robots.iteritems():
                       if currentLocation.aimingAt(robot.pose, settings.PASS_AIM_THRESHOLD):
                           aimTargets.append(robotId)
            # store
            e = ballPossessionChangeEvent(timeStamp, currentEngaged, currentOwner, self.location, aimTargets)
            trace('%s', (str(e)))
            self.buffer.set(e, timeStamp)
        # evaluate the events in recent history (shot/pass attempt, or just lost the ball)
        # note: evaluation must NOT be performed only on state change, because we should be able to determine the 
        # intent quite quickly after ball disengage (especially a successful shot on goal)
        self.evaluate(timeStamp)
    
    def evaluate(self, timeStamp):
        # inspect recent events (end of buffer)
        t0 = time.time()
        idx = self.buffer.time2idx(timeStamp - 12.0)
        trace("t=%.3f idx=%s", timeStamp, str(idx))
        teamNames = (self.statistics.teams[0]['teamName'], self.statistics.teams[1]['teamName'])
        currentEvent = self.buffer[idx]
        while currentEvent != None:
            # need to inspect the event?
            age = timeStamp - currentEvent.timeStamp 
            alreadyEvaluated = self.evaluated[str(currentEvent)]

            # main evaluation sequence, only look at disengage events (and context if needed, of course)
            if (currentEvent.engaged == False) and (not alreadyEvaluated) and (idx > 0):
            
                # prepare
                previousEvent = self.buffer[idx-1]
                robotId = previousEvent.owner.robot
                teamId = previousEvent.owner.team
                teamName = teamNames[teamId]
                nextEventExists = False
                nextEvent = None
                if idx < len(self.buffer) - 1:
                    nextEvent = self.buffer[idx+1]
                    nextEventExists = True
                trace("idx=%d L=%d currentEvent=%s previousEvent=%s haveNext=%d", idx, len(self.buffer), currentEvent, previousEvent, nextEventExists)
                
                # helper functions to classify this event
                
                def checkSelfPass():
                    # the moment of making a self pass is the moment of disengaging the ball
                    if (currentEvent.engaged == False
                            and nextEventExists == True
                            and nextEvent.engaged == True 
                            and previousEvent.engaged == True
                            and previousEvent.owner == nextEvent.owner):
                        elapsed = nextEvent.timeStamp - previousEvent.timeStamp
                        distance = (nextEvent.location - previousEvent.location).r()
                        if elapsed < settings.SELF_PASS_TIMEOUT: # but not after too much time has elapsed
                            self.statistics.teams[teamId]['selfPasses'] += 1
                            events.info("robot %d of team %s performed a self-pass over %.1fm" % 
                                        (robotId, teamName, distance), 
                                        level=2, timeStamp=currentEvent.timeStamp)
                            return True
                        return True # this long-duration self-pass will not count towards any other statistic ... better to stop trying to inspect it
                    return False
                    
                def checkShotOnGoal():
                    if (currentEvent.engaged == False
                            and previousEvent.engaged == True
                            and currentEvent.location.aimingAtGoal(settings.GOAL_DX_THRESHOLD)
                            and (age > settings.SHOOT_TIMEOUT)):
                        self.statistics.teams[teamId]['shootAttempts'] += 1
                        events.info("robot %d of team %s attempted a shot on goal" % 
                                    (robotId, teamName), 
                                    level=2, timeStamp=currentEvent.timeStamp)
                        return True
                    return False

                def checkPassAttempt():
                    # the moment of making a pass attempt is the moment of disengaging the ball
                    # then, we can see if there were friendly robots in the aiming cone ('aimTargets')
                    if (currentEvent.engaged == False
                            and previousEvent.engaged == True
                            and len(currentEvent.aimTargets) > 0):
                        return True
                    return False
                    
                def checkPassSuccess():
                    # the moment of making a succesfull pass is the moment of receiving the ball
                    if (checkPassAttempt()
                            and nextEventExists == True
                            and nextEvent.engaged == True 
                            and previousEvent.owner.robot != nextEvent.owner.robot
                            and previousEvent.owner.team == nextEvent.owner.team
                            and nextEvent.owner.robot in currentEvent.aimTargets):
                        elapsed = nextEvent.timeStamp - previousEvent.timeStamp
                        distance = (nextEvent.location - previousEvent.location).r()
                        if elapsed < settings.REGULAR_PASS_TIMEOUT:
                            self.statistics.teams[teamId]['passAttempts'] += 1
                            self.statistics.teams[teamId]['passes'] += 1
                            events.info("robot %d of team %s made a successful pass to robot %d over %.1fm" % 
                                        (robotId, teamName, nextEvent.owner.robot, distance), 
                                        level=2, timeStamp=nextEvent.timeStamp)
                            return True
                    return False

                def checkPassFail():
                    # pass is registered as failed if either timeout has expired, or opponent steals the ball
                    # calling sequence matters: 
                    # * first, check for shot on goal (to not falsely count lob over teammember as failed pass attempt)
                    # * then check for successful pass
                    # * then failed pass
                    # * finally ball steal
                    if (checkPassAttempt()
                            and age >= settings.REGULAR_PASS_TIMEOUT
                            and (not checkShotOnGoal())):
                        self.statistics.teams[teamId]['passAttempts'] += 1
                        # it is not necessarily the fault of the robot making the pass, 
                        # but it is the only straightforward scapegoat so tough luck!
                        events.info("robot %d of team %s failed to make a pass" % 
                                    (robotId, teamName), 
                                    level=2, timeStamp=currentEvent.timeStamp)
                        return True
                    return False

                def checkBallSteal():
                    if (currentEvent.engaged == False
                            and nextEventExists == True
                            and nextEvent.engaged == True 
                            and previousEvent.owner.team != nextEvent.owner.team):
                        elapsed = nextEvent.timeStamp - previousEvent.timeStamp
                        if elapsed < settings.BALL_STEAL_TIMEOUT:
                            self.statistics.teams[nextEvent.owner.team]['ballSteals'] += 1
                            events.info("robot %d of team %s has stolen the ball" % 
                                        (nextEvent.owner.robot, teamNames[nextEvent.owner.team]), 
                                        level=2, timeStamp=nextEvent.timeStamp)
                            return True
                    return False

                # try evaluate the current disengage event
                if checkSelfPass() or checkShotOnGoal() or checkPassSuccess() or checkPassFail() or checkBallSteal():
                    # event has been classified, prevent counting it again
                    self.evaluated[str(currentEvent)] = True
                else:
                    # cannot classify this event (yet)
                    # check if we should stop trying alltogether
                    if age > 10 and currentEvent.engaged == False:
                        events.info("failed to classify disengage event: %s" % 
                                    (str(currentEvent)), 
                                    level=4, timeStamp=currentEvent.timeStamp)
                        self.evaluated[str(currentEvent)] = True # prevent re-evaluation and repeating this failure message
                    
            # next iteration
            idx += 1
            currentEvent = self.buffer[idx]

