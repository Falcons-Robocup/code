#!/usr/bin/env python
#
# Plot tracing.
#
# FALCONS // Jan Feitsma, November 2017


import sys, os
from glob import glob
import argparse
sys.path.append("/home/robocup/falcons/code/packages/coachCommands")
from newest_logdir import newest_logdir

from tracePlotShooting import tracePlotShooting
from tracePlotBall import tracePlotBall
from tracePlotLatency import tracePlotLatency



def guessRobots(logdir):
    result = []
    for robotId in range(1,7):
        if len(glob(logdir + '/trace_A%d_wm.txt' % (robotId))):
            result.append(robotId)
    return result
    


# command line interface
if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='plot relevant tracing values over time for several use cases')
    parser.add_argument('--tStart', help='start timestamp', type=str, default=None)
    parser.add_argument('--tEnd', help='end timestamp', type=str, default=None)
    parser.add_argument('-r', '--robots', help='which robots to analyze', action='append', type=int, default=[])
    # TODO --live to watch files as they grow (like tail -f) ? maybe a bit too complex for too little usability gain.
    parser.add_argument('--mode', required=True, help='what group of data to plot', default=None, choices=['motion', 'ball', 'shooting', 'dutycycle', 'latency'])
    parser.add_argument('files', metavar='file', help='data files', type=str, nargs='*')
    # TODO: make a wrapper and a usage note for downloading and plotting data immediately after a match (half)
    args       = parser.parse_args()

    # if files not given, then take all from newest logdir
    if len(args.files) == 0:
        logdir = newest_logdir()
        args.files = glob(logdir + "/*_[dp]trace*.txt")
        if len(args.files) == 0:
            raise Exception("no trace files found in folder " + logdir)

    # construct
    if args.mode == "shooting":
        data = tracePlotShooting(args)
    if args.mode == "ball":
        data = tracePlotBall(args)
    if args.mode == "latency":
        data = tracePlotLatency(args)
    # TODO other modes

    # load
    data.load(args.files)
    
    # plot
    data.plot()

