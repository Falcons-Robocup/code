# Copyright 2017-2019 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
from robotScenarioBase import *

# measure shoot accuracy by various methods
#
# method A: repeatedly try to hit the goal post
#   * the robot will produce a shot from wherever it finds the ball
#   * standard motionPlanning::shootAtTarget is used
#   * optionally a fixed (x,y) coordinate can be given to move towards first, before shooting
#   * success rate has to be determined manually: was the goal post hit, or missed (or half)
#


# settings
GOAL_POST_X = 1.06 # true for Locht goal, assuming it is placed symmetrically
GOAL_POST_Y = 9
SHOOT_POSITIONS = [6, 5, 4, 3]

def _singleShot(x, y, x0, y0, callback):    
    while True:
        # wait until we see the ball
        while not robot.seeBall():
            sleep(1)
        # get ball
        robot.getBall()
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to target position
        if y0 != None:
            x0 = float(x0)
            y0 = float(y0)
            phi = robot.calculateTargetPhi(x0, y0)
            move(x0, y0, phi)
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to shooting position
        if y != None:
            x = float(x)
            y = float(y)
            phi = robot.calculateTargetPhi(x, y)
            robot.move(x, y, phi)
        # check ball possession
        if robot.hasBall():
            break;
    # execute the behavior
    callback()
    sleep(2)

def _repeater(x, y, x0, y0, callback):
    while True:
        _singleShot(x, y, x0, y0, callback)


def shootAccuracyPhi(fromX=None, fromY=None, x0=None, y0=None, goalX=GOAL_POST_X, goalY=GOAL_POST_Y):
    """
    Repeatedly shoot at the goalpost.
    Optionally, drive towards given position (x,y) to shoot from.
    Optionally, drive via position (x0,y0), to achieve systematic rotation with ball.
    """
    def executor():
        # aim and shoot via motionPlanning::shootAtTarget (blocking tst service)
        robot.shootAt(float(goalX), float(goalY))
    _repeater(fromX, fromY, x0, y0, executor)


def shootAccuracyZ(fromX=None, fromY=None, x0=None, y0=None, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    Repeatedly perform a lobshot at the middle of goal bar.
    Optionally, drive towards given position (x,y) to shoot from.
    Optionally, drive via position (x0,y0), to achieve systematic rotation with ball.
    """
    def executor():
        # aim and shoot via motionPlanning (blocking tst service)
        robot.lobShotAt(float(goalX), float(goalY), float(z))
    _repeater(fromX, fromY, x0, y0, executor)

def straightShotAccuracy(fromY=SHOOT_POSITIONS, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    from given positions, perform a straight shot  at the middle of goal bar.
    """
    def executor():
        # aim and shoot via motionPlanning::shootAtTarget (blocking tst service)
        robot.shootAt(float(goalX), float(goalY), float(z))
    
    for y in fromY:
        # shoot 3 times from the same position
        for _ in range(3):
            _singleShot(0, y, None, None, executor)

def lobShotAccuracy(fromY=SHOOT_POSITIONS, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    from given positions, perform a lobshot  at the middle of goal bar.
    """
    def executor():
        # aim and shoot via motionPlanning (blocking tst service)
        robot.lobShotAt(float(goalX), float(goalY), float(z))
    
    for y in fromY:
        # shoot 3 times from the same position
        for _ in range(3):
            _singleShot(0, y, None, None, executor)
