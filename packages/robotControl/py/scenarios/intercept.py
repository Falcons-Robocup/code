# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import math
import random
import argparse
import logging
from FalconsCoordinates import RobotPose


# settings
DEFAULT_ACTION_RADIUS = 2.0
DEFAULT_CIRCLE_RADIUS = 4.0
MAIN_DOCSTRING = """
    Automated single-robot intercept test. 
    Robot will choose a position on a circle, continuously attempting to intercept the ball and pass to next robot. 
    Includes a fallback getball in case ball bounces off.
"""
ARGS_DOCSTRING = """    Arguments:
    * circle radius, default 4.0
    """



# scenario entry point with parsed argument list and exposed docstring
def intercept(robot, circleradius=DEFAULT_CIRCLE_RADIUS):
    settings = _parse_arguments([])
    settings.circleradius = float(circleradius)
    interceptor = _Interceptor(robot, settings)
    interceptor.run()
intercept.__doc__ = MAIN_DOCSTRING + ARGS_DOCSTRING



def _parse_arguments(argv):
    parser = argparse.ArgumentParser(description=MAIN_DOCSTRING)
    parser.add_argument('-a', '--actionradius', help='zone/action radius: in case intercept fails and ball is within this radius, just do a getball fallback', type=float, default=DEFAULT_ACTION_RADIUS)
    parser.add_argument('-c', '--circleradius', help='home position circle radius on which robot default positions are set', type=float, default=DEFAULT_CIRCLE_RADIUS)
    parser.add_argument('-t', '--target', help='pass target (default: next robot)', type=float, nargs=2, default=None)
    parser.add_argument('-n', '--targetnoise', help='aim given amount of meters at a random side next to the target', type=float, default=0.0)
    parser.add_argument('-w', '--dontwait', help='do not wait with intercepting until previous robot has the ball', action='store_true')
    parser.add_argument('-q', '--quiet', help='suppress output', action='store_true')
    parser.add_argument('--home', help='home position (x,y), default calculated based on available robots and circleradius', type=float, nargs=2, default=None)
    parser.add_argument('-i', '--index', help='home position index to choose (starting count at 1), default calculate based on available robots', type=int, nargs=2, default=None)
    parser.add_argument('-r', '--robot', help='robot ID to use (intended only for simulation)', type=int, default=None)
    parser.add_argument('--ignore', help='robots to be ignored', type=int, nargs='+', default=[1])
    return parser.parse_args(argv)



class _Interceptor():
    def __init__(self, robot, settings=None):
        self.robot = robot
        self.settings = settings or _parse_arguments([])
        self.otherRobotHasBall = False
        self.state = None
        # TODO: port some of the circular-positioning functionality to robot.friends? to clean up this class and enable reuse elsewhere

    def activeRobots(self):
        # ignore r1, if it is present, because it can never contribute
        return [r for r in self.robot._ws.activeRobots() if not r in self.settings.ignore]

    def calculateRobotIndex(self):
        # optional overrule
        if self.settings.index != None:
            idx0 = self.settings.index[0] - 1
            n = self.settings.index[1]
        else:
            # default: get active robots and figure out index of this robot
            a = self.activeRobots()
            while not self.robot._robotId in a: # init robustness
                self.robot.sleep(0.1)
                a = self.activeRobots()
            n = len(a)
            idx0 = a.index(self.robot._robotId)
        return (idx0, n)

    def calcCirclePos(self, robotIdx, numRobots, radius=3, center=(0,0)):
        """
        Helper function to distribute robot positions on a circle.
        """
        gamma = 2*math.pi / numRobots
        x = radius * math.cos(gamma * robotIdx) + center[0]
        y = radius * math.sin(gamma * robotIdx) + center[1]
        rz = gamma * robotIdx - math.pi
        return (x, y, rz)

    def home(self):
        if self.settings.home == None:
            # default: position on a circle
            (idx0, n) = self.calculateRobotIndex()
            (x, y, rz) = self.calcCirclePos(idx0, n, self.settings.circleradius)
        else:
            # optional overrule
            (x, y) = self.settings.home
            rz = math.pi * 0.5
        # face the ball if possible
        b = self.robot._ws.getBallPosition()
        if b:
            rz = math.atan2(b.y - y, b.x - x)
        # execute
        self.setState('repositioning / waiting')
        self.robot.move(x, y, rz)

    def canStartIntercept(self):
        # optional overrule, to let robot immediately enter its intercept action
        if self.settings.dontwait:
            return True
        # robot should never stand idle if ball is closeby
        if self.ballCloseBy():
            return True
        # check if previous robot has the ball
        (idx0, n) = self.calculateRobotIndex()
        a = self.activeRobots()
        otherIdx = a[(idx0-1) % n]
        # wait for the pass (state change in ball possession)
        # robot should not intercept when other robot is still turning for instance
        otherRobotHadBall = self.otherRobotHasBall
        self.otherRobotHasBall = self.robot._ws.hasBall(otherIdx)
        return self.otherRobotHasBall == False and otherRobotHadBall == True

    def determineTarget(self, noise=None):
        # optional overrule
        if self.settings.target:
            (x, y) = self.settings.target
            rz = 0
        else:
            # calculate nominal position of next robot
            (idx0, n) = self.calculateRobotIndex()
            a = self.activeRobots()
            otherIdx = a[(idx0+1) % n]
            (x, y, rz) = self.calcCirclePos(idx0+1, n, self.settings.circleradius)
        otherPos = RobotPose(x, y, rz)
        # add noise?
        if noise:
            # add noise to RCS x (perpendicular)
            ownPos = self.robot._ws.getRobotPosition()
            ownPos.Rz = math.atan2(y - ownPos.y, x - ownPos.x) # face target
            otherPosRcs = otherPos.transform_fcs2rcs(ownPos)
            # offset RCS x in a random direction
            r = random.randint(0, 1)
            otherPosRcs.x += (r * 2 - 1) * noise
            # back to FCS
            otherPos = otherPosRcs.transform_rcs2fcs(ownPos)
        return (otherPos.x, otherPos.y) # ignore Rz

    def canPass(self):
        # compare current position of next robot with nominal
        nominalTarget = self.determineTarget()
        (idx0, n) = self.calculateRobotIndex()
        a = self.activeRobots()
        if len(a) == 1:
            return True
        otherIdx = a[(idx0+1) % n]
        otherPos = self.robot._ws.getRobotPosition(otherIdx)
        delta = otherPos - RobotPose(*nominalTarget)
        return delta.xy().size() < 0.3

    def ballCloseBy(self):
        return self.robot.ballCloseBy(distance=self.settings.actionradius)

    def setState(self, state):
        # only write state change
        if self.state != state:
            # write to RDL eventlog
            #os.system('export TURTLE5K_ROBOTNUMBER=' + str(self.settings.robot) + ' ; frun diagnostics sendEvent INFO "' + state + '" > /dev/null')
            # TODO: need a diagnostics.py interface instead of above
            # write to stdout
            logging.info(state)
        self.state = state

    def run(self):
        # iterate
        self.done = False
        while not self.done:
            # move to starting position, facing ball, with coarse tolerances
            self.home()
            # wait until robot can start his intercept/getBall attempt
            if self.canStartIntercept():
                # get the ball, preferably via intercept
                while not self.robot._ws.hasBall() and not self.done:
                    if self.ballCloseBy():
                        self.setState('getball fallback')
                        self.robot.getBall() # blocking
                    else:
                        self.setState('intercepting')
                        self.robot.interceptBall() # blocking (with not-so-obvious RUNNING/FAILED criteria -> see mp code)
                    # note: good weather behavior: ball comes into the action radius while the robot
                    # is continuously intercepting on it, until pass/fail, so the getBall
                    # fallback should only start after intercept returns FAILED due to the ball moving away
                # other robot might still be repositioning
                while not self.canPass() and not self.done:
                    self.setState('waiting to pass')
                    self.robot.sleep(0.1)
                # pass to next robot and sleep a while, to prevent directly chasing the ball
                self.setState('pass')
                self.robot.passTo(*self.determineTarget(self.settings.targetnoise))
                self.robot.sleep(0.5)
            else:
                # sleep a bit
                self.robot.sleep(0.1)
            # check if robot went offline
            self.done = self.robot._robotId not in self.activeRobots()


