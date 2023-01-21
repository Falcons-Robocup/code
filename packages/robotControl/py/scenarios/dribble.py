# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
# Originally this was the script for Robocup World Championship 2018, tech challenge run 2
# Conditions:
#   one robot, wifi on, normal MSL field and lighting
#   4 fixed obstacles
#   NOTE: since the space between obstacles is small, and we get really close to the goal area, we may need to disable obstacle avoidance!


OBSTACLE_Y_COORDINATES = [1.8, 3.6, 5.4, 7.2]
DEFAULT_START_Y = -1.0
DEFAULT_DX = 0.9
BACK_DRIBBLE_SPEED = 1.0
BACK_DRIBBLE_DURATION = 3.0
BACK_STEP_SPEED = 0.5
BACK_STEP_DURATION = 0.5



def _bhWorkaround(robot):
    # helper function for not properly dribbling, not yet having a (good) implementation of self-pass
    # officially, robot may not hold the ball dribbling for longer than X meters
    # the workaround is to drop the ball and ensure robot is not touching it
    robot.disableBallHandlers()
    robot.velocitySetpoint(0, -BACK_STEP_SPEED, 0, BACK_STEP_DURATION)
    robot.getBall()


def dribble(robot, numRoundTrips=3, withBackDribble=True, bhWorkaround=True, y0=DEFAULT_START_Y, dx=DEFAULT_DX):
    """
    Test dribble slalom around obstacles, using the original 'techchallenge run 2' scenario.
    Preparation: 4 static obstacles on specific spots.
    Arguments:
    * number of iterations (default 3)
    * boolean to optionally include back-dribble (default enabled)
    * boolean to explicitly drop the ball conform dribble distance regulations (default enabled; should be removed when self-pass is implemented)
    * initial y coordinate (official scenario was on penalty spot, but that is not default anymore)
    * x lane half-width
    Example (techchallenge configuration):
        scenario dribble 3 1 1 -6.0 1.0
    """
    # TODO: optionally disable obstacle avoidance?
    # TODO: tune down pathPlanning?
    phiY = 1.57
    # construct a sequence of waypoints
    wayPoints = []
    sign = 1
    for y in OBSTACLE_Y_COORDINATES:
        wayPoints.append((sign * dx, y))
        sign = -1 * sign
    wayPoints.append((0, y+dx))
    for y in reversed(OBSTACLE_Y_COORDINATES):
        wayPoints.append((sign * dx, y))
        sign = -1 * sign
    # TC run specification starts at own penalty spot; but during development typically other half is occupied 
    robot.move(0, y0, phiY)
    # ball is expected to be placed on (0,0), move close to it so we see it
    # (in simulation, ensure ball is teleported)
    robot.move(0, -1, phiY)
    robot.getBall()
    for i in range(numRoundTrips):
        for pos in wayPoints:
            robot.move(*pos)
            if bhWorkaround:
                _bhWorkaround(robot)
        # last move back to (0,0) is special
        if withBackDribble:
            # aim away from (0,0)
            pos = robot._ws.getRobotPosition()
            robot.faceTowards(pos.x * 2, pos.y * 2)
            # dribble backwards
            robot.velocitySetpoint(0, -BACK_DRIBBLE_SPEED, 0, BACK_DRIBBLE_DURATION)
        else:
            # final regular waypoint
            robot.move(0, 0)
        if bhWorkaround:
            _bhWorkaround(robot)

