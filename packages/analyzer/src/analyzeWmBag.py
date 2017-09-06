""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analyze the worldmodel stream in a bag file as if done live on coach, but faster.
# Useful for testing, debugging and post-match analysis.
#
# Jan Feitsma, 2017-04-27



import sys,os
import time
import argparse
from fnmatch import fnmatch
# TODO: put matchlog in a new package 'logging'
sys.path.append("/home/robocup/falcons/code/packages/visualizer/src/oldpy")
from matchlog import MatchLogBagged
# TODO: next should also be nicer
sys.path.append("/home/robocup/falcons/code/packages/coachCommands")
from newest_logdir import newest_logdir
import roslib
roslib.load_manifest('analyzer') 
from analyzeWorldModel import AnalyzeWorldModel 

            
TEAM_WM_TOPIC = "/teamA/g_worldmodel_team"            


def guessBagFile():
    logdir = newest_logdir()
    # See which .bag files there are 
    bagfiles = []
    actfiles = []
    for f in os.listdir(logdir):
        if fnmatch(f, '*.bag'):
            bagfiles.append(logdir + '/' + f)
    assert(len(bagfiles) == 1)
    bagfile = bagfiles[0]
    return bagfile
        
        
if __name__ == '__main__':
    # argument parsing
    parser     = argparse.ArgumentParser(description='analyze worldModel stream from bag file')
    parser.add_argument('bagfile', help='bag file', nargs='?', default=None)
    # TODO option to interpret a robot stream? tricky, need to glue several topics together
    args       = parser.parse_args()
    
    # load the bag file
    bagfile = args.bagfile
    if bagfile == None:
        bagfile = guessBagFile()
    matchlog = MatchLogBagged(bagfile)
    
    # stimulate
    a = AnalyzeWorldModel()
    for d in matchlog.data:
        if d[1] == TEAM_WM_TOPIC:
            a.feed(d[2], d[0])
            a.analyze()
    
    # display results
    a.display()

