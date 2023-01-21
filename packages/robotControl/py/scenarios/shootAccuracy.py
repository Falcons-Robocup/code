# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
# measure shoot accuracy by various methods




# settings
GOAL_POST_X = 1.06 # true for Locht goal, assuming it is placed symmetrically
GOAL_POST_Y = 9
GOAL_BAR_Z = 1.05
DEFAULT_DISTANCE_RZ = 3.0
DEFAULT_DISTANCE_Z = 3.0
DEFAULT_EXTRA_PASS_DISTANCE = 2.0


# helper functions

def _moveBeforeShot(robot, x, y, x0, y0):
    while True:
        # wait until we see the ball
        while not robot.seesBall():
            robot.sleep(1)
        # get ball
        robot.getBall()
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to fly-in position
        if y0 != None:
            robot.move(float(x0), float(y0))
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to shooting position
        if y != None:
            robot.move(float(x), float(y))
        # check ball possession
        if robot.hasBall():
            break


def _repeater(robot, numIterations, x, y, x0, y0, shootFunction):
    iteration = 0
    while iteration < numIterations:
        iteration += 1
        _moveBeforeShot(robot, x, y, x0, y0)
        shootFunction()
        robot.sleep(2)


def shootAccuracy(robot, numIterations=5, goalX=0, goalY=GOAL_POST_Y):
    """
    Generic shooting scenario: repeatedly fetch the ball and shoot at the goal/target.
    Arguments:
    * number of iterations (default 5)
    * optional non-standard goal target coordinates (x,y)
    """
    def shootFunction():
        robot.shootAt(float(goalX), float(goalY), 0.0)
    _repeater(robot, int(numIterations), None, None, None, None, shootFunction)


def shootAccuracyRz(robot, numIterations=5, distance=DEFAULT_DISTANCE_RZ, x0=None, y0=None, goalX=GOAL_POST_X, goalY=GOAL_POST_Y, extraPassDistance=DEFAULT_EXTRA_PASS_DISTANCE):
    """
    Repeatedly pass to the goalpost from a fixed distance.
    (Shooting would be too powerful.)
    Arguments:
    * number of iterations (default 5)
    * distance in meters (default 3.0)
    * optional fly-in position (x,y) to achieve systematic rotation with ball
    * optional non-standard goal post target coordinates (x,y)
    * optional extra pass distance, to increase pass strength a bit
    """
    def shootFunction():
        robot.passTo(float(goalX), float(goalY) + float(extraPassDistance))
    fromX = float(goalX)
    fromY = float(goalY) - float(distance)
    _repeater(robot, int(numIterations), fromX, fromY, x0, y0, shootFunction)


def shootAccuracyZ(robot, numIterations=5, distance=DEFAULT_DISTANCE_Z, lob=True, x0=None, y0=None, goalX=0, goalY=GOAL_POST_Y, z=GOAL_BAR_Z):
    """
    Repeatedly perform a lobshot at the middle of goal bar from a fixed distance.
    Arguments:
    * number of iterations (default 5)
    * distance in meters (default 3.0)
    * boolean to use lob shot (default use lob shot)
    * optional fly-in position (x,y) to achieve systematic rotation with ball
    * optional non-standard goal bar target coordinates (x,y,z)
    """
    lob = bool(lob)
    def shootFunction():
        if lob:
            robot.lobShotAt(float(goalX), float(goalY), float(z))
        else:
            robot.shootAt(float(goalX), float(goalY), float(z))
    fromX = float(goalX)
    fromY = float(goalY) - float(distance)
    _repeater(robot, int(numIterations), fromX, fromY, x0, y0, shootFunction)

