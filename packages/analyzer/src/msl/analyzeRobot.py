""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 
# FALCONS // Jan Feitsma, August 2017



# python includes
# none

# package includes
from utils import Pose
from datastream import dataStream


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time



# TODO smoothener: splines or Kalman?
BUFFER_TIMEOUT = 15.0


class analyzeRobot():
    def __init__(self):
        self.id = None
        self.buffer = dataStream(Pose)

    def feedPose(self, pose, timeStamp):
        trace('poseInput id=%s t=%.3f pose=%s', self.id, timeStamp, pose)
        self.buffer.set(pose, timeStamp)
        self.cleanup(timeStamp)

    def getPose(self, timeStamp=1e99):
        # circumvent dataStream behavior
        if timeStamp < self.buffer.timeRange()[0]:
            # get would return default packet, we do not want that
            return None
        pose = self.buffer.get(timeStamp)
        trace('poseOutput id=%s t=%.3f pose=%s', self.id, timeStamp, pose)
        return pose

    def cleanup(self, timeStamp):
        # prevent datastream buffer from growing too large
        # not just for efficiency, but also to deal with time gaps between active play,
        # which should not trigger false teleports
        self.buffer.cleanup(timeStamp - BUFFER_TIMEOUT)

    def frozen(self, age=10.0):
        # determine max
        maxDr = 0.0
        maxDphi = 0.0
        currPos = self.buffer[-1]
        tEnd = self.buffer._tEnd
        for (t, pos) in self.buffer._data.iteritems():
            if 1e-3 * t + age > tEnd:
                deltaPos = pos - currPos
                maxDr = max(maxDr, deltaPos.r())
                maxDphi = max(maxDphi, abs(deltaPos.phi))
        # evaluate
        #TODO debug trace("frozen: n=%d")
        return (maxDr < 0.5) and (maxDphi < 0.1)

