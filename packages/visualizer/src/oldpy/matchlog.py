""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # Matchlog data interface
# will either load realtime or from .bag file
# 2015-03-22 JFEI creation
# 2016-02-02 JFEI change from bulk t_matchlog to many small channels
#


import subprocess
import time,datetime
from inspect import isfunction
from collections import defaultdict
import traceback
import rospy
import roslib; roslib.load_manifest('rosMsgs')
import rosbag
from rosMsgs.msg import *




def topic2typ(topic):
    """
    Determine and return type belonging to topic, according to name convention.
    Example: 
        /teamA/robot3/g_robotspeed
    will return type
        t_robotspeed
    """
    pos = topic.rfind('/') + 1
    bare_topic_name = topic[pos:]
    type_name = 't_' + bare_topic_name[2:]
    return eval(type_name) # assume it was imported via 'from rosMsgs.msg import *'


def getRobotTopics():
    """
    Figure out which topics to listen to by inspecting the C++ source file, 
    which then becomes the central point of definition.
    Ideally this would be resolved compile time, but this visualizer is python based, not yet C++. 
    For that we would have to switch over to new visualizer.
    """
    cpp_file = '/home/robocup/falcons/code/packages/facilities/diagnostics/src/cDiagnosticsReceiver.cpp'
    sys_cmd = "grep 'add<' " + cpp_file + " | awk '{print $3}' | tr -d '\";,)'"
    output = subprocess.check_output(sys_cmd, shell=True)
    topics = output.split()
    # TODO: add analysis topics (written by analyzer)
    return topics
    

def getTeamTopics():    
    # analysis topics (written by analyzer)
    return ["g_worldmodel", "g_ana_error", "g_ana_info", "g_ana_online", "g_diag_refbox"]


class MatchLog():
    def __init__(self):
        self.buffer = {} # (key: topic, value: msg) to be drawn
        self.topicToType = {}
        self.warned = {}
        # callbacks to be installed by visualizer
        self.genericCallback = lambda k, v: None
        self.drawCallbacks = defaultdict(lambda : None)
        # setup team topics
        self.topics = ["/teamA/" + t for t in getTeamTopics()]
        # setup per-robot topics
        for t in getRobotTopics():
            for robotNum in range(1,7):
                topic = "/teamA/robot%d/%s" % (robotNum, t)
                self.topics.append(topic)
                self.topicToType[topic] = topic2typ(t)
                
    def warnOnce(self, topic, msg):
        if not self.warned.has_key(topic):
            print "WARNING: don't know how to deal with messages from topic %s !" % (topic)
            self.warned[topic] = 1
        
    def processBuffer(self):
        """
        Visualize the contents of buffer.
        Only draw last message per topic, to not waste time transforming outdated data into pixels.
        """
        for (k,v) in self.buffer.items():
            try:
                # text
                self.genericCallback(k, v)
                # pixels
                cb = self.drawCallbacks[k]
                if isinstance(cb, tuple):
                    # pass bound arguments
                    args = [e for e in cb if not callable(e)]
                    cbs = [e for e in cb if callable(e)]
                    for cb_ in cbs:
                        cb_(v, *args)
                else:
                    if cb != None:
                        cb(v)
                    else:
                        # inform developer that he probably should install a visualizer callback
                        self.warnOnce(k, v)
            except:
                print "something went wrong when executing callback for topic %s, message follows" % k
                print v
                traceback.print_exc()
                pass
        self.buffer = {}
    
    def advance(self, t):
        """
        Advance to given timestamp (relative).
        In realtime visualization, this triggers the draw callbacks for the running buffer
        (actually, only the last entries per topic, to save performance).
        In post-mortem (bagged) visualization, some extra logic is required. There, this function is overridden.
        """
        self.processBuffer()
      

class MatchLogBagged(MatchLog):
    # self.data is an array of tuples (t1, topic, msg, t2), where
    #  * t1 is a posix timestamp (float), e.g. 1437281188.41
    #  * topic the topic on which the message was received
    #  * msg the message data struct
    #  * t2 a datetime converted time object (for display)
    def __init__(self, bagfile):
        MatchLog.__init__(self)
        print "loading bagfile ", bagfile
        # load all at once, can take a few seconds though
        bag = rosbag.Bag(bagfile)
        self.data = []
        first = True
        prev = 0
        for topic, msg, t in bag.read_messages():
            if first:
                self.t0 = t.to_time()
                first = False
            t1 = t.to_time()
            assert(t1 >= prev) # check that the data stream is ordered in time
            t2 = datetime.datetime.fromtimestamp(t1)
            self.data.append((t1, topic, msg, t2))
            prev = t1
        self.elapsed = t1 - self.t0
        print "done"
        print "  t_start:", str(self.data[0][3])
        print "  t_end  :", str(t2)
        print "  elapsed: %6.2f" % (self.elapsed)
        self.pointer = 0

    def advance(self, t):
        """
        Advance to given timestamp (relative).
        Since this is in post-mortem (bagged) visualization, we just fill the buffer based on the timestamps in data bag.
        """
        # translate relative to absolute time
        t = t + self.t0
        # dumb lookup to find latest message just before t
        # invariant: pointer is a valid index and data is not empty
        tpoint = self.data[self.pointer][0]
        if t < tpoint:
            # rewind to zero
            self.pointer = 0
            tpoint = self.data[self.pointer][0]
            self.buffer = {}
        while t > tpoint:
            if t - tpoint < 1.0:
                self.buffer[self.data[self.pointer][1]] = self.data[self.pointer][2]
            # advance until tpoint is just larger than t
            self.pointer += 1
            if self.pointer > len(self.data) - 1:
                self.pointer = len(self.data) - 1
                break
            tpoint = self.data[self.pointer][0]
        # one step back, make sure it is within bounds
        if self.pointer > 0:
            self.pointer -= 1
        if self.pointer > len(self.data) - 1:
            self.pointer = len(self.data) - 1
        # process buffer and clear it
        self.processBuffer()


class MatchLogRealTime(MatchLog):
    def __init__(self):
        MatchLog.__init__(self)
        rospy.init_node('matchlog_listener')
        self.t0 = time.time()
        # instantiate subscribers
        for topic in self.topics:
            rospy.Subscriber(topic, topic2typ(topic), self.callback, topic)
    def callback(self, msg, key):
        self.buffer[key] = msg



