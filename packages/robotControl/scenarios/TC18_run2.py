# Copyright 2018-2019 Martijn (Falcons)
# SPDX-License-Identifier: Apache-2.0
from robotScenarioBase import *
from math import pi


# Script for Robocup World Championship 2018, tech challenge run 2
# Conditions:
#   one robot, wifi on, normal MSL field and lighting
#   4 fixed obstacles
#   NOTE: since the space between obstacles is small, and we get really close to the goal area, need to disable obstacle avoidance!

# Goal:
#   Step 1: slalom dribble around them, without holding the ball for more than 3 meters
#   Step 2: backwards dribble for 1.5 meter, 3 times
#
# pseudocode step 1:
#   Get ball
#   dribble to waypoint
#   stop + release ballhandlers
#   move back to release ball
#   get ball
#   move to next waypoint



# helperfunctions
def _moveBack():
    ownPos = robot.ownPosition()
    x = ownPos.x
    y = ownPos.y
    phi = ownPos.Rz
    robot.disableBallHandlers()
    # in simulator releasing the ball does not work; teleport it manually? Then print target position
    if phi > 2:
        # looking backward, so moving back means moving forward in y
        robot.move(x, y+0.3, phi)
    else:
        robot.move(x, y-0.3, phi)
    robot.enableBallHandlers()
    robot.getBall()


# phi: 0 = 2pi = -x, pi = +x, 1.57 = +y, 4.71 = -y
def TC18_run2():
    # TODO: disable obstacle avoidance?
    # TODO: tune down pathPlanning?
    phiY = 1.57
    phiMY = 4.71
    phiX = pi
    phiMX = 0
    # start at own penalty spot
    robot.move(0, -6, phiY)
    # ball is placed on 0,0, move close to it so we see it
    # teleport ball to 0,0 # manually type in simulator
    robot.move(0, -1, phiY)
    robot.getBall()
    robot.move(-1.2, 2.3,1.57)
    robot.move(-0.9, 2.3,1.57)
    _moveBack()	
    for i in range(0,3):
    #sequence starts here
        robot.move(-0.9, 2.3, 1.1)
        robot.move(0.9, 3.5, 1.1)
        robot.move(0.9, 3.5, 1.57)
        robot.move(0.9, 4.1, 1.57)
        _moveBack()
        robot.move(0.9, 4.1,2.67)
        robot.move(-0.9, 5.3,2.67)
        robot.move(-0.9, 5.3,1.57)
        robot.move(-0.9, 5.9,1.57)
        _moveBack()
        robot.move(-0.9, 5.9,1.1)
        robot.move(0.9, 7.1,1.1)
        robot.move(0.9, 7.1,1.57)
        robot.move(0.9, 7.7,1.57)
        _moveBack()
        robot.move(0.9, 8.2,1.57)
        robot.move(0.9, 8.2,3.14)
        robot.move(-0.9, 8.2,3.14)
        robot.move(-0.9, 8.2,4.71)
        robot.move(-0.9, 7.1,4.71)
        _moveBack()
        robot.move(-0.9, 7.1,5.81)
        robot.move(0.9, 5.9,5.81)
        robot.move(0.9, 5.9,4.71)
        robot.move(0.9, 5.3,4.71)
        _moveBack()
        robot.move(0.9, 5.3,3.61)
        robot.move(-0.9, 4.1,3.61)
        robot.move(-0.9, 4.1,4.71)
        robot.move(-0.9, 3.5,4.71)
        _moveBack()
        robot.move(-0.9, 3.5,5.81)
        robot.move(0.9, 2.3,5.81)
        robot.move(0.9, 2.3,4.71)
        robot.move(0.9, 1.7,4.71)
        _moveBack()
        robot.move(0.9, 1.1,4.71)
        robot.move(0.9, 1.1,3.14)
        robot.move(-0.9, 1.1,3.14)
        robot.move(-0.9, 1.1,1.57)
        robot.move(-0.9, 2.3,1.57)
        _moveBack()
    #x = -1.5
    #y = 1.1
    #robot.move(-1.5,1.1,phiY)
    #_moveBack()
    #robot.move(-1.5,2.9,phiY)
    #robot.move(1.5,2.9,phiY)
    #robot.move(1.5,4.7,phiY)
   # _moveBack()
#    for parcours in range(0,1): # TODO: real run: 3 or 6 times, waiting for response rule committee;
#        # initially went diagonally in between obstacles (TC18_run2_diagonal.py); robot struggles to find a path in between --> go straight in between, also easier to ensure we don't dribble more than 3m
#        TargetX = -1.5
#        TargetY = 1.1
#        robot.move(TargetX, TargetY, phiY)
#        _moveBack()
#        TargetX = -1.5
#        TargetY = 2.9
#        robot.move(TargetX, TargetY, phiY)
#        _moveBack()
#        for i in range(0,3):
#            TargetX = TargetX * -1
#            robot.move(TargetX, TargetY, phiY)
#            _moveBack()
#            TargetY = TargetY + 1.8
#            robot.move(TargetX, TargetY, phiY)
#            _moveBack()
#        # now we should be at 1.5, 8.3; move a bit away from the goal before crossing the field
#        robot.move(1.5, 8, phiX)
#        robot.move(-1.5, 8, phiX)
#        _moveBack() # now we dribble for 3.3 meter, assume nobody will notice
#        TargetX = -1.5
#        TargetY = 6.5
#        robot.move(TargetX , TargetY, phiMY)
#        _moveBack()
#        for i in range(0,3):
#            TargetX = TargetX * -1
#            robot.move(TargetX, TargetY, phiMY)
#            _moveBack()
#            TargetY = TargetY - 1.8
#            robot.move(TargetX, TargetY, phiMY)
#            _moveBack()
#        robot.move(-1.5, 1.1, phiY) # move to starting position
#        _moveBack()
#    print "Step 1 parcours finished, starting step 2 backward dribble \n"
#    robot.move(-1.5, 0, phiY) # make sure we are angled correctly so we don't dribble backward out of the field
#    ownPos = robot.ownPosition()
#    TargetX = ownPos.x
#    TargetY = ownPos.y
#    phi = ownPos.Rz
#    for i in range(0,3):
#        robot.setVelocity(0, -1, 0, 2)
#        _moveBack()
#        sleep(1) # after getball wait to ensure we really have the ball before driving backwards
#    print "Step 2 backward dribble finished \n"


