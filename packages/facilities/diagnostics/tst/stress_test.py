""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python

PKG = 'diagnostics'
import roslib; roslib.load_manifest(PKG)

import rospy
import sys
import unittest
from collections import defaultdict
from threading import Thread, Lock
from time import sleep
from rosMsgs.msg import *
from FalconsTrace import trace
from cStringIO import StringIO

# import UDPsender
sys.path.append("/home/robocup/falcons/code/scripts")
from udpsend import UDPsender
# TODO msg diagEnum

# thread safety
_lock = Lock()


class TestStress(unittest.TestCase):

    def __init__(self, *args):
        trace("constructing TestStress object")
        unittest.TestCase.__init__(self)
        self.N = 100
        # list some topic (not necessarily all the same as defined in cDiagnosticsReceiver.cpp)
        self.topics = []
        self.subscribers = []
        self.topics.append((6, "g_diag_robot_control", t_diag_robot_control))
        self.topics.append((7, "g_vision_diag", t_vision_diag))
        self.callcount = defaultdict(lambda : 0)
        trace("construction complete")

    def udp_talker(self, robotnum, count, sleeptime):
        trace((robotnum, count, sleeptime))
        multicast = True
        sleep(1) # give some time for topic listeners to come online
        for tdef in self.topics:
            # TODO brigde C++/python
            port = 10000 + 1000*robotnum + tdef[0]
            u = UDPsender("224.16.32.74", port, multicast)
            # serialize message
            b = StringIO()
            msg = tdef[2]().serialize(b)
            # send repeatedly
            for i in range(count):
                u.burstSend(b.getvalue()) 
                sleep(sleeptime)
    
    def callback(self, data, topic):
        trace("callback triggered for topic " + topic)
        # data is irrelevant, we are just counting
        with _lock:
            self.callcount[topic] += 1
    
    def listen(self):
        trace("listening")
        for robotnum in range(1,7):
            for tdef in self.topics:
                topic = ("/robot%d/" % (robotnum)) + tdef[1]
                trace("subscribing to topic " + topic)
                self.subscribers.append(rospy.Subscriber(topic, tdef[2], self.callback, topic))
        # wait a while
        trace("spinning")
        rospy.spin()
        
    def verify(self):
        good = True
        for robotnum in range(1,7):
            for tdef in self.topics:
                topic = ("/robot%d/" % (robotnum)) + tdef[1]
                trace("checking : topic " + topic + " published %d times" % (self.callcount[topic]))
                if self.callcount[topic] != self.N:
                    good = False
        if not good:
            sys.exit(1) # a bit blunt, but a mere 'assert' does not escalate to the MAIN thread ... 
        
    def runTest(self):
        trace("runTest")
        threads = []
        # setup: instantiate diagnostics receiver -- this is handled via stress_test.xml
        # setup: instantiate topic listeners
        thread = Thread(target = self.listen)
        thread.start()
        threads.append(thread)
        trace("listener thread has started")
        # execution: send a whole lot of UDP packets all at once from different threads
        sleeptime = 0.0000
        for robotnum in range(1,7):
            thread = Thread(target = self.udp_talker, args = (robotnum, self.N, sleeptime))
            thread.start()
            trace("robot thread %d has started" % (robotnum))
            threads.append(thread)
        # wait for all threads to finish
        sleep(3)
        # don't bother with joining threads -- the talkers have finished but rospy.spin is blocking the listener thread
        # so just return, the test framework doesn't mind
        self.verify()


if __name__ == '__main__':
    import rostest
    rospy.init_node('listener', anonymous=True)
    rostest.rosrun(PKG, 'diag_stress_test', TestStress)
    
    

