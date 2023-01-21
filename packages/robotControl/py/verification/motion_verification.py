# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

# Jan Feitsma, March 2020

# Script to let the robot move around in order to accuratly determine several robot motion parameters, such as wheel slip and encoder feedback accuracy scaling.
#
# For description and usage hints, execute with '-h'


from __future__ import print_function, division
import sys, os
import argparse
import falconspy
import rtdb2tools
from robotLibrary import RobotLibrary
from twister import Twister



def parse_arguments():
    parser = argparse.ArgumentParser(description="Verify robot motion. For details and examples, see wiki/diagnostics/motion.")
    parser.add_argument('-x', help='x position to stand', type=float, default=0.0)
    parser.add_argument('-y', help='y position to stand', type=float, default=0.0)
    parser.add_argument('-n', '--numRepeats', help='number of repeats', type=int, default=3)
    parser.add_argument('--dof', help='direction (degree of freedom) to move', type=str, choices=['x', 'y', 'Rz'], default='Rz')
    return parser.parse_args()


class MotionRangeTwister():
    def __init__(self, args):
        # setup control and state reader
        self.rl = RobotLibrary(args.robot)
        # get ball and move to starting position
        self.homePosition = (args.x, args.y, 0.0)
        self.reset()
        # other settings
        self.dof = args.dof
        self.v = None
        self.accRange = None
        self.setRange() # sensible defaults for v and accRange
        # setup twister, reuse connected RCI
        self.twister = Twister(args.robot, rci=self.rl._rci)
        self.twister.dof = self.dof
        self.twister.duration = 1.0
        self.twister.sleep = 0.5
        self.twister.repeats = args.numRepeats
        self.twister.disableVisionLoc = True

    def setRange(self):
        # default acceleration range and speed setpoint
        self.v = 0.5
        step = 0.2
        if (self.dof == 'Rz'):
            self.v = 2.0
            step = 0.5
        self.accRange = [step * n for n in range(6,20)] # bh_verification
        self.accRange = [step*2 * n for n in range(10,20)] # motion_verification

    def reset(self):
        self.rl.move(*self.homePosition)

    def info(self, msg):
        # write to RDL eventlog
        os.system('frun diagnostics sendEvent INFO "' + msg + '"')
        # write to stdout
        print(msg, end='')
        sys.stdout.flush() # TODO remove, can use print() argument flush=True when migrating to python3

    def run(self):
        # run
        for a in self.accRange:
            self.reset()
            isRz = (self.dof == 'Rz')
            self.info("testing {0} a={1:.2f}{2}2 at v={3:.2f}{2}...".format(self.dof, a, ['m/s', 'rad/s'][isRz], self.v))
            self.twister.speed = self.v
            self.twister.accelerationLimit = a
            self.twister.run()


if __name__ == '__main__':
    args = parse_arguments()

    robot = rtdb2tools.guessAgentId()
    if robot == 0 or robot == None:
        raise RuntimeError("Error: could not determine robot ID, this script should run on a robot")
    args.robot = robot

    rt = MotionRangeTwister(args)
    rt.run()

