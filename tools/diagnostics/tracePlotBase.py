#!/usr/bin/env python
#
# FALCONS // Jan Feitsma, November 2017


from tracePlotRobot import *
import sys


class tracePlotBase:

    def __init__(self, settings):
        self.settings = settings
        self.robots = {}
        for robotId in range(1,9):
            self.robots[robotId] = tracePlotRobot(robotId)

    def load(self, files):
        print "loading %d files ..." % (len(files)), 
        for f in files:
            for robotId in range(1,9):
                if ('A%d' % (robotId)) in f:
                    self.robots[robotId].load(f, self.settings)
                    sys.stdout.write(".")
                    sys.stdout.flush()
        # performance optimization: convert to numpy arrays
        for robotId in range(1,9):
            self.robots[robotId].wm.prep()
            self.robots[robotId].bh.prep()
            self.robots[robotId].mp.prep()
            self.robots[robotId].pp.prep()
            self.robots[robotId].br.prep()
            self.robots[robotId].bt.prep()
            self.robots[robotId].bm.prep()
            self.robots[robotId].bd.prep()
            self.robots[robotId].vis.prep() 
            # TODO this is very error prone, improve design!
        # calculate t0: first timestamp of loaded data
        t0 = 1e99
        for robotId in range(1,9):
            t0 = min(t0, self.robots[robotId].t0)
        t0 = datetime.datetime.fromtimestamp(t0 + EPOCH) # as datetime, for convertTimeStr
        # limit time window
        timeRange = [0, 1e99]
        if self.settings.tStart != None:
            timeRange[0] = convertTimeStr(self.settings.tStart, t0)
        if self.settings.tEnd != None:
            timeRange[1] = convertTimeStr(self.settings.tEnd, t0)
        for robotId in range(1,9):
            self.robots[robotId].limit(timeRange)
        print " done"

