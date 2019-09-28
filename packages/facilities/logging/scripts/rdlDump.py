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
    parser = argparse.ArgumentParser(description="Parse and dump contents of a Robot Data Logging (RDL) file")
    parser.add_argument("file", help="path to RDL file")
    parser.add_argument("-a", "--agent", action='append', type=int, default=[], help="which agents to show, default all")
    parser.add_argument("-k", "--key", action='append', type=str, default=[], help="which keys to show (comma-separated), default all")
    parser.add_argument("-t", "--timestamp", type=str, default="all", help="timestamp of interest (for now: age relative to start of log)")
    parser.add_argument("-d", "--deltatime", type=float, default=1, help="time window in seconds around given timestamp")
    #TODO revise behavior # parser.add_argument("-r", "--raw", action="store_true", help="only dump values, not metadata")
    # TODO: some formatting option(s)?
    args = parser.parse_args()

    # check if file exists
    if not os.path.exists(args.file):
        print "Error: '%s' not found." % (args.file)
        exit()

    # expand default arguments, resolve time frame, etc.
    ageMin = 0
    ageMax = 1e9
    if args.agent == []:
        args.agent = range(0,10)
    if args.timestamp != "all":
        # for now: age -- TODO: parse human-readable timestamp (e.g. '2019-02-28,23:01:33.221551' or substrings without date/microseconds)
        ageMin = float(args.timestamp) - args.deltatime
        ageMax = float(args.timestamp) + args.deltatime

    # go through the frames
    rdlFile = RDLFile(args.file)
    rdlFile.parseRDL()
    frameCounter = 0
    for frame in rdlFile.frames:
        # determine whether this frame needs to be shown
        inspectFrame = True
        if (frame.age < ageMin or frame.age > ageMax):
            inspectFrame = False
        # print?
        if inspectFrame:
            # header?
            print "frame=%d/%d age=%.3fs" % (frameCounter, len(rdlFile.frames), frame.age)
            # TODO include human-readable timestamp, without date
            for agent in frame.data.keys():
                showAgent = False
                if len(args.agent) == 0:
                    showAgent = True
                else:
                    showAgent = (agent in args.agent)
                if showAgent:
                    for key in sorted(frame.data[agent].keys()):
                        showKey = False
                        if len(args.key) == 0:
                            showKey = True
                        else:
                            showKey = (key in args.key)
                        if showKey:
                            item = frame.data[agent][key]
                            line = "  %2d %s %30s -> %s" % (item.agent, "LS"[item.shared], item.key, str(item.value))
                            print line
        # next
        frameCounter += 1

