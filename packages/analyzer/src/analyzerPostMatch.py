""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analysis node on coach laptop.
# Listens to all diagnostics topics, adds a few new analysis topics next to it.
#
# Jan Feitsma, 2016-03-05



import sys,os
import roslib
import rospy
roslib.load_manifest('analyzer') 
from rosMsgs.msg import *
import time
from fnmatch import fnmatch
# TODO: put matchlog in a new package 'logging'
sys.path.append("/home/robocup/falcons/code/packages/visualizer/src/oldpy")
from matchlog import MatchLogBagged
# TODO: next should also be nicer
sys.path.append("/home/robocup/falcons/code/packages/coachCommands")
from newest_logdir import newest_logdir



class AnalyzerPostMatch():
    def __init__(self, bagfile = None):
        # load bag file
        if bagfile == None:
            d = newest_logdir()
            for f in os.listdir(d):
                if fnmatch(f, '*.bag'):
                    bagfile = d + "/" + f
        self.bagfile = bagfile
        self.matchlog = MatchLogBagged(bagfile)
        
    def getErrors(self):
        result = []
        for d in self.matchlog.data:
            if d[1] == "/teamA/g_ana_error":
                result.append(d)
        return result

    def getTopics(self):
        result = {}
        for d in self.matchlog.data:
            result[d[1]] = 1
        return result.keys()
    
    def getActiveRobots(self):
        result = []
        topics = self.getTopics()
        for teamchar in ["A", "B"]:
            for robotnum in range(1,7):
                topic = "/team%s/robot%d/g_diag_worldmodel" % (teamchar, robotnum)
                if topic in topics:
                    result.append(teamchar + str(robotnum))
        return result
    
    def getMessage(self, topic, t):
        """
        Return last message on specified topic before t, 
        where t is relative time since start of logging.
        In case t is negative, then count backwards from the end.
        """
        if t < 0:
            t += self.matchlog.elapsed
        for d in self.matchlog.data[::-1]:
            if (d[1] == topic) and (d[0] - self.matchlog.t0 < t):
                return d[2]
        return None
            
    def getCollisions(self):
        print "TODO getCollisions"
        return []
    
    def summary(self):
        active = self.getActiveRobots()
        if len(active):
            print "active robots: " + " ".join(self.getActiveRobots())
        else:
            print "NO ACTIVE ROBOTS"
        errors = self.getErrors()
        if len(errors):
            print "ERRORS: "
            for e in errors:
                print "  " + str(e[3]) + " : " + e[2].error
        else:
            print "no errors occurred"
            
            
        
if __name__ == '__main__':
    if len(sys.argv) > 1:
        bagfile = sys.argv[1]
        a = AnalyzerPostMatch(bagfile)
    else:
        a = AnalyzerPostMatch()
    a.summary()
    if len(sys.argv) > 3:
        print ""
        topic = sys.argv[2]
        t = float(sys.argv[3])
        print a.getMessage(topic, t)
        
