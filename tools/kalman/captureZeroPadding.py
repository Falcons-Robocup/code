#!/usr/bin/env python
#
# JFEI 2016-05-03 
# Given a worldModel capture file, pad encoder data with zero values.
# This should have a positive influence on Kalman behavior (sort of 'anchoring')

 
import sys, os
import argparse


if __name__ == '__main__':

    # Argument parsing.
    parser     = argparse.ArgumentParser(description='extend a capture with encoder values')
    parser.add_argument('-f', '--freq', help='desired frequency of encoder input', default=30.0, type=float)
    parser.add_argument('inputfile', help='input file', type=str, default=None)
    parser.add_argument('outputfile', help='output file', type=str, default=None)
    args       = parser.parse_args()

    # Open output file for writing
    o = file(args.outputfile, 'w')

    # Work through input file
    dt = 1.0 / args.freq
    lastTimeStamp = 0
    currTimeStamp = 0
    for line in file(args.inputfile).readlines():
        words = line.split()
        nextTimeStamp = float(words[0])
        while currTimeStamp + dt < nextTimeStamp:
            currTimeStamp += dt
            o.write("%10.6f  encoder  %10.6f  %10.6f  %10.6f\n" % (currTimeStamp, 0.0, 0.0, 0.0))
        currTimeStamp = nextTimeStamp
        o.write(line)
    o.close()

