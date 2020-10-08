""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

# Jan Feitsma, March 2020

# Robot will continuously intercept around current position.
#
# For description and usage hints, execute with '-h'


import sys, os
import time
import logging, signal
logging.basicConfig(level=logging.INFO)
import math, random
import argparse
import falconspy
import rtdb2tools
from robotLibrary import RobotLibrary
from worldState import WorldState
from FalconsCoordinates import *



def parse_arguments():
    parser = argparse.ArgumentParser(description="""Automated single-robot intercept test. Robot will choose a position in a circle, continuously attempting to intercept the ball and pass to next robot. Includes a fallback getball in case ball bounces off. See also: wrapper script interceptCircle.py.""")
    parser.add_argument('-a', '--actionradius', help='zone/action radius: in case intercept fails and ball is within this radius, just do a getball fallback', type=float, default=2.0)
    parser.add_argument('-c', '--circleradius', help='home position circle radius on which robot default positions are set', type=float, default=4.0)
    parser.add_argument('-t', '--target', help='pass target (default: next robot)', type=float, nargs=2, default=None)
    parser.add_argument('-n', '--targetnoise', help='aim given amount of meters at a random side next to the target', type=float, default=0.0)
    parser.add_argument('-w', '--dontwait', help='do not wait with intercepting until previous robot has the ball', action='store_true')
    parser.add_argument('-q', '--quiet', help='suppress output', action='store_true')
    # TODO use option 'active' intercept?
    parser.add_argument('--home', help='home position (x,y), default calculated based on available robots and circleradius', type=float, nargs=2, default=None)
    parser.add_argument('-i', '--index', help='home position index to choose (starting count at 1), default calculate based on available robots', type=int, nargs=2, default=None)
    parser.add_argument('-r', '--robot', help='robot ID to use (intended only for simulation)', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('--ignore', help='robots to be ignored', type=int, nargs='+', default=[1])
    return parser.parse_args()


def calcCirclePos(robotIdx, numRobots, radius=3, center=(0,0)):
    """
    Helper function to distribute robot positions on a circle.
    """
    gamma = 2*math.pi / numRobots
    x = radius * math.cos(gamma * robotIdx) + center[0]
    y = radius * math.sin(gamma * robotIdx) + center[1]
    phi = gamma * robotIdx - math.pi
    return (x, y, phi)


class Interceptor():
    def __init__(self, settings):
        self.settings = settings
        self.rl = RobotLibrary(settings.robot, joystick=False)
        self.ws = WorldState(settings.robot)
        self.ws.startMonitoring()
        self.otherRobotHasBall = False
        # setup logging
        self.state = None
        self.logger = self.initializeLogger()
        if settings.quiet:
            self.logger.setLevel(logging.NOTSET)
        # setup signal handler for proper shutdown
        self.done = False
        signal.signal(signal.SIGINT, self.signalHandler)

    def signalHandler(self, signal, frame):
        self.done = True
        self.ws.stopMonitoring()
        self.rl.shutdown()
        # TODO: this is not yet working as intended...

    def initializeLogger(self):
        """
        Setup the logging environment
        """
        log = logging.getLogger()  # root logger
        log.setLevel(logging.INFO)
        format_str = '%(asctime)s.%(msecs)03d - %(levelname)-8s - r' + str(self.settings.robot) + ' - %(message)s'
        date_format = '%Y-%m-%dT%H:%M:%S'
        formatter = logging.Formatter(format_str, date_format)
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        log.handlers = [] # clear
        log.addHandler(stream_handler)
        return logging.getLogger(__name__)

    def activeRobots(self):
        # ignore r1, if it is present, because it can never contribute
        return [r for r in self.ws.activeRobots() if not r in self.settings.ignore]

    def calculateRobotIndex(self):
        # optional overrule
        if self.settings.index != None:
            idx0 = self.settings.index[0] - 1
            n = self.settings.index[1]
        else:
            # default: get active robots and figure out index of this robot
            a = self.activeRobots()
            while not self.settings.robot in a: # init robustness
                time.sleep(0.1)
                a = self.activeRobots()
            n = len(a)
            idx0 = a.index(self.settings.robot)
        return (idx0, n)

    def calculateHomePosition(self):
        # optional overrule
        if self.settings.home != None:
            (x, y) = self.settings.home
            rz = math.pi * 0.5
        else:
            # default: position on a circle
            (idx0, n) = self.calculateRobotIndex()
            (x, y, rz) = calcCirclePos(idx0, n, self.settings.circleradius)
        # face the ball if possible
        b = self.ws.getBallPosition()
        if b:
            rz = math.atan2(b.y - y, b.x - x)
        return (x, y, rz)

    def canStartIntercept(self):
        # optional overrule
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
        self.otherRobotHasBall = self.ws.hasBall(otherIdx)
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
            (x, y, rz) = calcCirclePos(idx0+1, n, self.settings.circleradius)
        otherPos = RobotPose(x, y, rz)
        # add noise?
        if noise:
            # add noise to RCS x (perpendicular)
            ownPos = self.ws.getRobotPosition()
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
        otherPos = self.ws.getRobotPosition(otherIdx)
        delta = otherPos - RobotPose(*nominalTarget)
        return delta.xy().size() < 0.3

    def ballCloseBy(self):
        bd = self.ws.ballDistance()
        return bd != None and bd < self.settings.actionradius

    def setState(self, state):
        # only write state change
        if self.state != state:
            # write to RDL eventlog
            os.system('export TURTLE5K_ROBOTNUMBER=' + str(self.settings.robot) + ' ; frun diagnostics sendEvent INFO "' + state + '" > /dev/null')
            # write to stdout?
            logging.info(state)
        self.state = state

    def run(self):
        # iterate
        while not self.done:
            # move to starting position, facing ball, with coarse tolerances
            homePos = self.calculateHomePosition()
            self.setState('repositioning / waiting')
            self.rl.move(*homePos, xyTol=0.1, rzTol=0.05)
            # wait until robot can start his intercept/getBall attempt
            if self.canStartIntercept():
                # get the ball, preferably via intercept
                while not self.ws.hasBall() and not self.done:
                    if self.ballCloseBy():
                        self.setState('getball fallback')
                        self.rl.getBall() # blocking
                    else:
                        self.setState('intercepting')
                        self.rl.interceptBall() # blocking (with not-so-obvious RUNNING/FAILED criteria -> see mp code)
                    # note: good weather behavior: ball comes into the action radius while the robot
                    # is continuously intercepting on it, until pass/fail, so the getBall
                    # fallback should only start after intercept returns FAILED due to the ball moving away
                # other robot might still be repositioning
                while not self.canPass() and not self.done:
                    self.setState('waiting to pass')
                    time.sleep(0.1)
                # pass to next robot and sleep a while, to prevent directly chasing the ball
                self.setState('pass')
                self.rl.passTo(*self.determineTarget(self.settings.targetnoise))
                time.sleep(0.5)
            else:
                # sleep a bit
                time.sleep(0.1)
            # check if robot went offline
            self.done = self.settings.robot not in self.activeRobots()


def main(args):
    interceptor = Interceptor(args)
    interceptor.run()


if __name__ == '__main__':
    args = parse_arguments()
    if args.robot == 0 or args.robot == None:
        raise RuntimeError("Error: could not determine robot ID, this script should run on a robot")
    main(args)

