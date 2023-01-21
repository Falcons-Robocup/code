# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

# Jan Feitsma, March 2020

# Verify robot capability to hold the ball in its ball handlers,
# by using increasing setpoints until it loses the ball.
#
# TODO migrate this to blockly. But then we need to be able to limit acceleration on ROBOT_VELOCITY_SETPOINT... how?
#
# For description and usage hints, execute with '-h'


from __future__ import print_function, division
import sys
import argparse
import falconspy
import rtdb2tools
from robotLibrary import RobotLibrary
from worldState import WorldState # not yet python3 compatible (import error: rospkg)
from motion_verification import MotionRangeTwister


def parse_arguments():
    parser = argparse.ArgumentParser(description="Verify robot capability to hold the ball in its ball handlers, by using increasing setpoints until it loses the ball. For details and examples, see wiki/diagnostics/ballHandling.")
    parser.add_argument('-x', help='x position to stand', type=float, default=0.0)
    parser.add_argument('-y', help='y position to stand', type=float, default=0.0)
    parser.add_argument('-n', '--numRepeats', help='number of repeats', type=int, default=3)
    parser.add_argument('--dof', help='direction (degree of freedom) to move', type=str, choices=['x', 'y', 'Rz'], default='Rz')
    return parser.parse_args()


class BallHandlingVerification(MotionRangeTwister):
    def __init__(self, args):
        # setup control and state reader
        MotionRangeTwister.__init__(self, args)
        self.ws = WorldState(args.robot)
        self.ws.startMonitoring() # pitfall, easy to forget ... better to automatically start? (is there a use case for constructing WorldState but not starting the update thread)
        # other settings
        self.minimumSuccessRate = 0.5
        self.numRepeats = args.numRepeats
        # customize twister
        self.twister.repeats = 1 # re-evaluate ball possession after each twist
        self.twister.disableVisionLoc = False

    def reset(self):
        self.rl.getBall()
        self.rl.move(*self.homePosition)

    def findAccelerationLimit(self):
        # find acceleration where ball cannot be held anymore
        result = None
        for a in self.accRange:
            self.reset()
            isRz = (self.dof == 'Rz')
            self.info("testing {0} a={1:.2f}{2}2 at v={3:.2f}{2}...".format(self.dof, a, ['m/s', 'rad/s'][isRz], self.v))
            self.twister.speed = self.v
            self.twister.accelerationLimit = a
            self.twister.run()
            countBallLost = 0
            for n in range(self.numRepeats):
                self.twister.run()
                if not self.ws.hasBall():
                    countBallLost += 1
                    self.reset()
            countOk = self.numRepeats - countBallLost
            print(" successrate={:d}/{:d}".format(countOk, self.numRepeats))
            if countOk == 0:
                break
            if countOk / self.numRepeats > self.minimumSuccessRate:
                result = a
        return result


if __name__ == '__main__':
    args = parse_arguments()

    robot = rtdb2tools.guessAgentId()
    if robot == 0 or robot == None:
        raise RuntimeError("Error: could not determine robot ID, this script should run on a robot")
    args.robot = robot

    b = BallHandlingVerification(args)
    a = b.findAccelerationLimit()
    if a == None:
        print("Something went wrong in the test, no valid acceleration limit was returned.")
    else:
        print("Maximum acceleration limit with good success rate: {:.2f}{:s}2".format(a, ['m/s', 'rad/s'][args.dof == 'Rz']))

