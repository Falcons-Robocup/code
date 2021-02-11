# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
import os
import argparse
from rdlLib import RDLFile


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Filter a Robot Data Logging (RDL) file")
    parser.add_argument("inputfile", help="path to input RDL file")
    parser.add_argument("outputfile", help="path to output RDL file")
    # TODO parser.add_argument("-a", "--agent", action='append', type=int, default=[], help="agents to keep, default all")
    # TODO parser.add_argument("-K", "--removekey", action='append', type=str, default=[], help="which keys to remove (comma-separated), default none")
    parser.add_argument("-t", "--timestamp", type=str, default="all", help="timestamp of interest (for now: age relative to start of log)")
    parser.add_argument("-d", "--deltatime", type=float, default=1.0, help="time window in seconds around given timestamp")
    # TODO: make -t and -d nicer, do it consistently for rdlDump as well
    # TODO: some formatting option(s)?
    args = parser.parse_args()

    # check if file exists
    if not os.path.exists(args.inputfile):
        print("Error: '%s' not found." % (args.inputfile))
        exit()

    # expand default arguments, resolve time frame, etc.
    ageMin = 0
    ageMax = 1e9
    #if args.agent == []:
    #    args.agent = range(0,10)
    if args.timestamp != "all":
        # for now: age -- TODO: parse human-readable timestamp (e.g. '2019-02-28,23:01:33.221551' or substrings without date/microseconds)
        ageMin = float(args.timestamp) - args.deltatime
        ageMax = float(args.timestamp) + args.deltatime

    # setup output file and load input entirely into memory (TODO: make nicer)
    if args.inputfile == args.outputfile:
        print("Error: input and outputfiles may not be equal.")
        exit()
    rdlInputFile = RDLFile(args.inputfile)
    rdlInputFile.parseRDL(ageMin, ageMax)
    rdlOutputFile = RDLFile(args.outputfile)
    
    # go through the frames
    for frame in rdlInputFile.frames:
        # determine whether this frame needs to be kept
        if (frame.age >= ageMin and frame.age <= ageMax):
            # correct frame.age w.r.t. new creation timestamp
            frame.age = frame.age - ageMin
            frame.raw_frame[0] = frame.age # TODO: make nicer
            # keep
            rdlOutputFile.frames.append(frame)
            # TODO: satisfy other options (agent, key)
            # TODO: minimize code duplication w.r.t. rdlDump

    # tweak the header
    rdlOutputFile.header = rdlInputFile.header
    rdlOutputFile.header.creation = rdlInputFile.header.creation + ageMin

    # write output file
    rdlOutputFile.write()
    
