""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python3


import os, sys
import argparse
import datetime
import falconspy
from rdlLib import RDLFile
from rdl_adapters import convertTimeStamp



def parse_arguments():
    descriptionTxt = """Display events contained in RDL file.
"""
    exampleTxt = """Example:
$ eventlog ~/falcons/data/internal/logfiles/20190925_opponent_goal_subset_r1.rdl
1569437729.786 - INFO  - r3 : kicking with speed 180.0 and height 57.5
1569437732.286 - INFO  - r1 : extending keeperFrame RIGHT
1569437732.286 - INFO  - r1 : keeper frame extend RIGHT
1569437733.619 - INFO  - r1 : extending keeperFrame LEFT
1569437734.653 - INFO  - r1 : extending keeperFrame RIGHT
1569437734.635 - INFO  - r6 : Ignored out-of-bounds target x:  0.29, y:-10.55
1569437736.173 - INFO  - r0 : refbox: COMM_STOP
1569437738.908 - INFO  - r0 : refbox: COMM_GOAL_OPP
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('rdlfile', help='RDL file to inspect', default=None, nargs='?')
    return parser.parse_args()


class Event():
    def __init__(self, e, agent):
        self.agent = agent
        self.timestamp = convertTimeStamp(e['timeStamp'])
        self.severity = ["INFO", "WARN", "ERROR"][e['severity']]
        self.message = e['message']
        # ignore funcName, fileName and lineNumber, one can easily find this information if desired (which is rare)
    def __repr__(self):
        timestampStr = datetime.datetime.fromtimestamp(self.timestamp).strftime("%Y-%m-%d,%H:%M:%S.%f")
        return "{:19s} - {:5s} - r{:d} : {:s}".format(timestampStr, self.severity, self.agent, self.message)


def find_events(rdl_file_or_frames):
    result = []
    # allow two kind of arguments: rdl file name (string) or rdl frames (list)
    if isinstance(rdl_file_or_frames, str):
        rdlfile = rdl_file_or_frames
        # load RDL
        rdl = RDLFile(rdlfile)
        rdl.parseRDL()
        rdlframes = rdl.frames
    else:
        rdlframes = rdl_file_or_frames
    # helper
    seen = {}
    def handle(events, agent):
        for e in events:
            if not str(e) in seen:
                result.append(Event(e, agent))
                seen[str(e)] = True
    # iterate over frames
    key = "EVENT_LIST"
    for frame in rdlframes:
        for agent in range(0, 20):
            if agent in frame.data:
                agentData = frame.data[agent]
                if key in agentData:
                    handle(agentData[key].value, agent)
    return result


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    if args.rdlfile:
        # run on given RDL
        for e in find_events(args.rdlfile):
            print(e)
    else:
        # live rtdb monitor
        print("TODO implement live monitor mode ...")

