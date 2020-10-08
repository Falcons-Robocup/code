#!/usr/bin/env python3
#
# Jan Feitsma, december 2019


import sys, time
import argparse
from testGrabs import TestGrabs, report



if __name__ == '__main__':
    # Argument parsing.
    descriptionTxt = """Regression-test the multiCam software on ALL available grabs. Write a small report. Example partial output:

    test  1/54: r1 20181030_212155 ... robotpos=n/a                  ball=n/a                 #obst=0  bposs=False
    test  2/54: r1 20190223_135008 ... robotpos=(-0.00,-0.00, 0.807) ball=(-0.27,-0.74, 0.00) #obst=0  bposs=False
    test  3/54: r1 20190427_160053 ... robotpos=n/a                  ball=n/a                 #obst=0  bposs=False
    test  4/54: r2 20180619_214554 ... robotpos=n/a                  ball=n/a                 #obst=0  bposs=False
    test  5/54: r2 20180809_223758 ... robotpos=(-4.49,-7.50, 6.165) ball=(-3.26,-7.84, 0.00) #obst=1  bposs=False
    test  6/54: r2 20180809_223759 ... robotpos=n/a                  ball=n/a                 #obst=0  bposs=False
    test  7/54: r2 20180809_223800 ... robotpos=(-4.48,-7.52, 6.166) ball=(-3.26,-8.91, 0.00) #obst=1  bposs=False
    ...

Note: this test uses worldModel for the final (time-averaged) interpretation, since it is too tricky to reliably interpret the RTDB vision output buffers. Perhaps it would be worthwile to address this issue towards creating a vision-only option?

See also: testGrabs.py (to run a single set of grabs)."""
    parser     = argparse.ArgumentParser(description=descriptionTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-v', '--verbose', help='print each command/action', action='store_true')
    parser.add_argument('-m', '--make', help='first make the tools', action='store_true')
    parser.add_argument('-M', '--cleanmake', help='first clean make the tools', action='store_true')
    parser.add_argument('-t', '--timeout', help='timeout before a test iteration is ended', type=float, default=15.0)
    parser.add_argument('-b', '--block', help='instead of using timeout, block each iteration until multiCam GUI is shut down by user', action='store_true')
    parser.add_argument('-s', '--start', help='start at given grab index', type=int, default=1)
    # TODO: speedup getting a lock etc. such that iteration timeout can be reduced? 10 seconds is apparently not enough
    args       = parser.parse_args()

    # setup TestGrabs object, suppress output
    tester = TestGrabs(args.verbose, True)

    # build?
    if args.make or args.cleanmake:
        tester.make(args.cleanmake)

    # create the queue of grabs to be tested
    testqueue = []
    for robot in range(1, 20):
        timestamps = tester.findGrabTimestamps(robot)
        for timestamp in timestamps:
            testqueue.append((robot, timestamp))

    # run the queue
    print("")
    count = args.start - 1
    for (robot, timestamp) in testqueue[(args.start-1):]:
        count += 1
        grabs = tester.getGrabs(robot, timestamp)
        print("{n}test {:2d}/{:2d}: r{} {} ... {n}".format(count, len(testqueue), robot, timestamp, n=["","\n"][args.verbose]), end="", flush=True)
        tester.run(robot, grabs, block=args.block)
        if not args.block:
            # wait a few seconds before shutting down
            time.sleep(args.timeout)
        tester.shutdown(exit=False)
        # inspect RTDB contents
        if args.verbose:
            print("{n}test {:2d}/{:2d}: r{} {} result: ".format(count, len(testqueue), robot, timestamp, n=["","\n"][args.verbose]), end="")
        print(report(robot))

    # cleanup
    tester.shutdown(exit=True)

