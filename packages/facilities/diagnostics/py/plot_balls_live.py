# Copyright 2020-2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import os, sys
import time, signal
import argparse
import threading
from ball_data import BallData
from plot_balls import BallPlotter, NUMROBOTS
import falconspy
import falconsrtdb



def parse_arguments():
    descriptionTxt = """Plot ball tracking performance linked to standard playback slider.

See also plot_balls_rdl.py in case you already know the timestamp and need no scrolling capability.
"""
    exampleTxt = ""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    return parser.parse_args()


class LiveBallPlotter():
    def __init__(self, dataAdapter):
        self.dataAdapter = dataAdapter
        # setup BallPlotter
        self.ballPlotter = BallPlotter()
        # setup redrawing upon change
        dataAdapter.onChange = self.redraw()
    def redraw(self):    
        self.ballPlotter.plot(self.dataAdapter)


class LiveBallDataAdapter(BallData):
    def __init__(self):
        BallData.__init__(self)
        self.rtdbStore = falconsrtdb.FalconsRtDBStore()
        self.key2function = {}
        self.key2function["BALL_CANDIDATES_FCS"] = self.feedBallCandidates
        self.key2function["BALLS"] = self.feedBallResults
        self.key2function["DIAG_WORLDMODEL_LOCAL"] = self.feedBallDiagnostics
        self.key2function["MATCH_STATE"] = self.feedMatchState
        self.lastItem = {} # to detect change in state
        self.onChange = None
        # since we spawn a thread while also having a GUI active,
        # we must setup a signal handler for proper shutdown
        self.ok = True
        #signal.signal(signal.SIGINT, self.signalHandler) TODO can remove?
        # continuously monitor RTDB contents
        t = threading.Thread(target=self.run)
        t.start()

    def shutdown(self):
        self.ok = False

    def signalHandler(self, signal, frame):
        self.shutdown()

    def run(self):
        dt = 1.0 / 30
        while self.ok:
            time.sleep(dt)
            self.tick()

    def tick(self):
        change = False
        # read various RTDB items from database
        for agent in range(0, NUMROBOTS+1):
            for key in self.key2function.keys():
                # no check on item.age(), to prevent data disappearing when playback is paused
                item = self.rtdbStore.get(agent, key, timeout=False)
                if item != None:
                    # check if item is new
                    akey = (agent, key)
                    if self.lastItem.has_key(akey) and self.lastItem[akey] != item:
                        # feed the item to self
                        item.agent = agent
                        self.key2function[key](item)
                        change = True
                        self.lastItem[akey] = item
        # trigger redraw
        if change and self.onChange:
            self.onChange()


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # run
    dataAdapter = LiveBallDataAdapter()
    ballPlotter = LiveBallPlotter(dataAdapter)
    ballPlotter.run()
    dataAdapter.shutdown()

