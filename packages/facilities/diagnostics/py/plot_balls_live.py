""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python


import os, sys
import time, signal
import argparse
import threading
from ball_data import BallData
from plot_balls import BallPlotter, NUMROBOTS
import falconspy
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH



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
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH)
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
                item = self.rtdb2Store.get(agent, key, timeout=False)
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

