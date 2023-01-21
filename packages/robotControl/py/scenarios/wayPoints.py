# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0

def wayPoints(robot, *args):
    """
    Given a sequence of positions (triples (x,y,phi)), move between them.
    Repeat forever.
    Example, to let the robot continuously cross center circles while facing center:
        scenario wayPoints 2 0 3.14 -2 0 0
    """
    wayPointsSleep(robot, 0, *args)
    

def wayPointsSleep(robot, duration, *args):
    """
    Given a sequence of positions (triples (x,y,phi)), move between them.
    Repeat forever.
    First argument is the duration in seconds to sleep in between.
    Example (same as above, but with a bit of sleep):
        scenario wayPointsSleep 1.0 2 0 3.14 -2 0 0
    """
    duration = float(duration)
    N = len(args) / 3
    it = 0
    while True:
        x = float(args[3*it])
        y = float(args[3*it+1])
        phi = float(args[3*it+2])
        robot.move(x, y, phi) # blocks
        # sleep?
        robot.sleep(duration)
        # goto next
        it += 1
        if it == N:
            it = 0
    
    
