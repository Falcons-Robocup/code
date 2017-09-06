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
    # yucky hacky to deal with an old inconsistency....
    if bare_topic_name == "g_diag_worldmodel":
        type_name = 't_diag_worldmodel'
    if bare_topic_name == "g_ana_event":
        type_name = 't_event'
    return eval(type_name) # assume it was imported via 'from rosMsgs.msg import *'

def topic2typStr(topic):
    pos = topic.rfind('/') + 1
    bare_topic_name = topic[pos:]
    type_name = 't_' + bare_topic_name[2:]
    # yucky hacky to deal with an old inconsistency....
    if bare_topic_name == "g_diag_worldmodel":
        type_name = 't_diag_worldmodel'
    if bare_topic_name == "g_ana_event":
        type_name = 't_event'
    return "rosMsgs/" + type_name

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
    # add analysis topics (written by analyzer)
    topics.append("g_ana_comm")
    return topics
    

def getTeamTopics():    
    # analysis topics (written by analyzer)
    return ["g_worldmodel_team", "g_ana_event", "g_ana_online", "g_ana_matchstate", "g_diag_refbox"]



class MatchLogBaggedPublisher():
    """
    This class can load a bag file and stimulate topics, so visualizer can playback post-mortem exactly the same as it would realtime.
    It needs a playback object to control time, speed and offset (slider etc).
    """
    def __init__(self, bagfile, upgrader):
        # initialize self
        rospy.init_node('matchlog_bagged_playback')
        self.frequency = 30.0
        self.upgrader = upgrader
        # topic blacklist
        self.blacklist = {}
        # load the bag file
        self.loadBagFile(bagfile)
        # setup topic publishers 
        self.setupTopics()
        
    def setupTopics(self):
        self.buffer = {} # (key: topic, value: msg) to be drawn
        self.warned = {}
        # setup team topics
        self.topics = ["/teamA/" + t for t in getTeamTopics()]
        # setup per-robot topics
        for t in getRobotTopics():
            for robotNum in range(1,7):
                topic = "/teamA/robot%d/%s" % (robotNum, t)
                self.topics.append(topic)
        self.publishers = {}
        for topic in self.topics:
            self.publishers[topic] = rospy.Publisher(topic, topic2typ(topic), queue_size=10)

    def setBlackList(self, topic):
        self.blacklist[topic] = True
        # also set it for other robots
        if "/robot" in topic:
            pos = topic.rfind('/') + 1
            bare_topic_name = topic[pos:]
            for robotNum in range(1,7):
                self.blacklist["/teamA/robot%d/%s" % (robotNum, bare_topic_name)] = True
        
    def processBuffer(self):
        """
        Send the contents of buffer over respective topics.
        Only process last message per topic, to not waste time transforming outdated data into pixels.
        """
        for (k,v) in self.buffer.items():
            # check blacklist
            if self.blacklist.has_key(k):
                continue
            # process the message
            try:
                if self.publishers.has_key(k):
                    # call upgrader to cast data to the applicable message type
                    msg = self.upgrader(v, topic2typStr(k))
                    # publish
                    self.publishers[k].publish(msg)
                else:
                    # blacklist the topic
                    print "WARNING: unknown topic: '%s'" % (k)
                    self.setBlackList(k)
            except:
                print "something went wrong when executing callback for topic %s, message follows" % k
                print v
                traceback.print_exc()
                # blacklist the topic
                self.setBlackList(k)
                pass
        self.buffer = {}
      
    def loadBagFile(self, bagfile):
        # self.data is an array of tuples (t1, topic, msg, t2), where
        #  * t1 is a posix timestamp (float), e.g. 1437281188.41
        #  * topic the topic on which the message was received
        #  * msg the message data struct
        #  * t2 a datetime converted time object (for display)
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
        self.tElapsed = t1 - self.t0
        self.tStart = self.data[0][0]
        self.tEnd = self.data[-1][0]
        print "done"
        print "  t_start:", str(self.data[0][3])
        print "  t_end  :", str(t2)
        print "  elapsed: %6.2f" % (self.tElapsed)
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

    def run(self, playback):
        done = False
        rate = rospy.Rate(self.frequency)
        dt = 1.0 / self.frequency
        while not done:
            # get timestamp from playback
            t = playback.updateTime(dt)
            # advance and publish
            self.advance(t)
            # sleep
            rate.sleep()
            if rospy.is_shutdown():
                done = True
            if t > self.tElapsed:
                done = True
            


