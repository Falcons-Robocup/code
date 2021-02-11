# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import os, sys
import argparse
from collections import OrderedDict
import falconspy
from ball_data import BallData
from plot_balls import BallPlotter, PlotBrowser, NUMROBOTS
import analyze_lib
from rdlLib import RDLFile
import rtdb2tools


def parse_arguments():
    descriptionTxt = """Plot ball tracking performance for given RDL and given timestamp.

See also plot_balls_live.py which updates live during playback.
"""
    exampleTxt = ""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-a', '--agent', help='agent ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument("-t", "--timestamp", type=float, required=True, help="timestamp of interest (age relative to start of log)")
    parser.add_argument("-w", "--timewindow", type=float, default=0.001, help="time window in seconds around given timestamp")
    parser.add_argument('rdlfile', help='extract data from RDL file instead of live RTDB', default=None, nargs='?')
    return parser.parse_args()


class RDLBallPlotter():
    def __init__(self, dataAdapter, agent):
        dataTimeout = 1.0
        self.fieldPlots = PlotBrowser('field plot')
        self.timelinePlots = PlotBrowser('timeline plot')
        self.timelinePlots.connect(self.fieldPlots) # sync browsing
        self.fieldPlots.defaultLegendLoc = 'lower right'
        self.timelinePlots.defaultLegendLoc = 'lower right'
        for age in dataAdapter.data.keys():
            balldata = dataAdapter.data[age]
            p = BallPlotter(show=False) # figure construction managed by PlotBrowser
            p.plot(balldata)
            self.fieldPlots.plots.append(p.fieldPlot)
            self.timelinePlots.plots.append(p.timelinePlot)
            p.setupEventCallback(self.fieldPlots.fig)
    def run(self):
        # start the browser (both figures will be shown)
        self.fieldPlots.select(0)
        self.fieldPlots.show()

class RDLBallDataAdapter():
    def __init__(self):
        self.data = OrderedDict() # key: age, value: BallData

    def load(self, rdlfile, timestamp, timewindow = 0.001):
        # create a list of BallData objects (one per age) which can be browsed
        # form age list
        frequency = 30.0
        ageList = [timestamp]
        c = 1
        while c / frequency < timewindow:
            ageList = [timestamp - c / frequency] + ageList + [timestamp + c / frequency]
            c += 1
        # maintain a list with latest RTDB item, to handle sparsity of RDL
        latestData = {}
        keys = ["BALL_CANDIDATES_FCS", "BALLS", "DIAG_WORLDMODEL_LOCAL"]
        # load RDL
        self.rdl = RDLFile(rdlfile)
        self.rdl.parseRDL()
        # iterate over frames
        c = 0 # index of ageList currently being constructed
        for frame in self.rdl.frames:
            # store as BallData
            if frame.age > ageList[c]:
                bd = BallData()
                key2function = {}
                key2function["BALL_CANDIDATES_FCS"] = bd.feedBallCandidates
                # it is not useful anymore to look at BALL_CANDIDATES_FCS,
                # since the plotter now browses through a few timestamps of tracker-associated data, 
                # instead of plotting point clouds from entire time ranges at once
                key2function["BALLS"] = bd.feedBallResults
                key2function["DIAG_WORLDMODEL_LOCAL"] = bd.feedBallDiagnostics
                # feed the items
                for agent in range(0, NUMROBOTS+1):
                    if agent in frame.data:
                        agentData = frame.data[agent]
                        for key in key2function.keys():
                            akey = str((agent, key))
                            if key in agentData and akey in latestData:
                                key2function[key](latestData[akey])
                                del latestData[akey] # prevent item being copied into next BallData
                # store
                self.data[ageList[c]] = bd
                # next
                c += 1
                if c >= len(ageList):
                    break
            # update latestData
            if(frame.age >= timestamp - timewindow - 1.0 and frame.age <= timestamp + timewindow):
                for agent in range(0, NUMROBOTS+1):
                    if agent in frame.data:
                        agentData = frame.data[agent]
                        # feed the items
                        for key in keys:
                            if key in agentData:
                                akey = str((agent, key))
                                latestData[akey] = agentData[key]


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name
    agent = args.agent
    if agent == 0:
        agent = analyze_lib.get_agent_from_rdl_filename(args.rdlfile)
    assert(os.path.isfile(args.rdlfile))
    # handle timestamp argument
    timestamp = 0
    timewindow = 1e9 # default show all
    if args.timestamp != "all":
        timestamp = float(args.timestamp)
        timewindow = args.timewindow
    # run
    dataAdapter = RDLBallDataAdapter()
    dataAdapter.load(args.rdlfile, timestamp, timewindow)
    ballPlotter = RDLBallPlotter(dataAdapter, agent)
    ballPlotter.run()

