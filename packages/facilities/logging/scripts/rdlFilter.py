""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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
        print "Error: '%s' not found." % (args.inputfile)
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
        print "Error: input and outputfiles may not be equal."
        exit()
    rdlInputFile = RDLFile(args.inputfile)
    rdlInputFile.parseRDL()
    rdlOutputFile = RDLFile(args.outputfile)
    
    # go through the frames
    for frame in rdlInputFile.frames:
        # determine whether this frame needs to be kept
        if (frame.age >= ageMin and frame.age <= ageMax):
            # keep
            rdlOutputFile.frames.append(frame)
            # TODO: satisfy other options (agent, key)
            # TODO: minimize code duplication w.r.t. rdlDump
            # TODO: correct frame.age w.r.t. new creation timestamp

    # tweak the header
    rdlOutputFile.header = rdlInputFile.header
    rdlOutputFile.header.duration = rdlOutputFile.frames[-1].age

    # write output file
    rdlOutputFile.write()
    
