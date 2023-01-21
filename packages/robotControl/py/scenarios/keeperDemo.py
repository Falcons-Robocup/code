# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import math


# Script for system demo: keeper.
# Also controls a shooter/attacker robot, which is expected to have sufficient shootAccuracy.
# TODO: migrate to blockly (?)
# See wiki for setup, scoring, etc.


# configuration
REPEATS        = 2 # number of times shooting left and right per distance
FIELD_SIZE_Y   = 9 # field size configuration, TODO use environmentField
DISTANCES      = [3, 6, 9] # from how far to shoot, tradeoff keeper response time versus aim interpretation
GETBALL_RADIUS = 2 # set to lower
AIM_FORWARD    = math.pi * 0.5 # nominal aim forward
AIM_X_LEFT     = -0.75 # x coordinate of left shot
AIM_X_RIGHT    = 0.75 # x coordinate of right shot
AIM_Z          = 0.5 # z coordinate of all shots
AIM_Y          = FIELD_SIZE_Y # y coordinate of all shots
AIM_TOLERANCE  = 0.01
FLYIN_OFFSET   = 0.3 # how far to drive back
FLYIN_SPEED    = 0.3 # how fast to drive back


class keeperDemo():
    """
    Demonstrate keeper ability to block shots.
    When the keeper (robot 1) runs this scenario, it will just go into keeper behavior mode.
    When an assisting robot is running this scenario, it will produce a series of shots.
    At the moment, no arguments are parsed.
    """

    def __init__(self, robot):
        self.robot = robot # store robot for further use
        if self.robot._robotId == 1:
            # TODO: automatically configure on isolated worldModelSync, so keeper cannot cheat by getting ball data from other robot
            self.robot.behavior("B_GOALKEEPER") # blocks forever
        else:
            self._shootAtKeeperSequence()


    # helper functions

    def _performShot(self, robotPos, shootTarget):
        while not self.robot.hasBall():
            # get the ball
            if self.robot.ballCloseBy(GETBALL_RADIUS):
                self.robot.getBall()
            self.robot.sleep(1)
        # to minimize sensitivity for shooting aim problems,
        # we perform a little fly-in move, or actually: a small step-back step-forward
        self.robot.move(*robotPos)
        robotPosFineAim = self.robot.faceTowards(shootTarget[0], shootTarget[1]) # TODO: fine AIM_TOLERANCE?
        # move backwards using speed, otherwise robot might start to rotate (Tokyo Drift),
        # which might negatively affect shooting accuracy, rendering this keeper evaluation useless ..
        self.robot.velocitySetpoint(0.0, -FLYIN_SPEED, 0.0, FLYIN_OFFSET / FLYIN_SPEED)
        self.robot.move(*robotPosFineAim) # TODO: fine AIM_TOLERANCE?
        # now any ballHandling arm errors should be minimized
        self.robot.shootAt(*shootTarget)
        self.robot.sleep(1)


    def _shootAtKeeperSequence(self):
        # main sequence for the attacker
        for dist in DISTANCES:
            pos = (0, FIELD_SIZE_Y - dist, AIM_FORWARD)
            for it in range(REPEATS):
                # shoot left and right from this position
                self._performShot(pos, (AIM_X_LEFT, AIM_Y, AIM_Z))
                self._performShot(pos, (AIM_X_RIGHT, AIM_Y, AIM_Z))


