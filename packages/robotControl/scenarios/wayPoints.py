# Copyright 2017-2019 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
from robotScenarioBase import *



def wayPoints(*args):
    """
    Given a sequence of positions (triples (x,y,phi)), move between them.
    Repeat forever.
    """
    wayPointsSleep(0, *args)
    

def wayPointsSleep(t, *args):
    """
    Given a sequence of positions (triples (x,y,phi)), move between them.
    Repeat forever.
    First argument is the amount of seconds to sleep in between.
    """
    t = float(t)
    N = len(args) / 3
    it = 0
    while True:
        x = float(args[3*it])
        y = float(args[3*it+1])
        phi = float(args[3*it+2])
        robot.move(x, y, phi) # blocks
        # sleep?
        sleep(t)
        # goto next
        it += 1
        if it == N:
            it = 0
    
    
