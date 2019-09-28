""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # Author: Erik Kouters
#

import os
import sys, time
import Tkinter as tk
import threading
from TrajectoryPlanner_2ndorder import TrajectoryPlanner

import roslib; roslib.load_manifest('pathPlanning') 
import rospy
#from rosMsgs.msg import t_robotspeed
from pathPlanning.srv import s_pathplanning_setpointGeneratorType, s_pathplanning_setpointGeneratorTypeResponse

setpointPublishing = True

t = TrajectoryPlanner()

def handle_call(srvdata):
    print "handling call", srvdata
    s_0 = srvdata.s_0
    v_0 = srvdata.v_0
    s_end = srvdata.s_end
    v_end = srvdata.v_end
    v_max = srvdata.v_max
    a_max = srvdata.a_max
    dt = srvdata.dt

    result = t.computeFullTrajectory(s_0, v_0, s_end, v_end, v_max, a_max)

    if result == -1:
        print "Not supported -- skipping."
        return s_pathplanning_setpointGeneratorTypeResponse(0.0)
    else:
        sampleRate = dt
        # Get the jerk at the next step
        newA = t.plot(sampleRate, result)[0]
        newV = t.computeVelocity(sampleRate, v_0, newA)
        newS = t.computeDisplacement(sampleRate, s_0, v_0, newA)
        return s_pathplanning_setpointGeneratorTypeResponse(newS, newV, newA, 0.0) # ignore jerk


if __name__ == "__main__":

    rospy.init_node("setpointGeneratorNode", anonymous=True)
    s = rospy.Service('getJerk', s_pathplanning_setpointGeneratorType, handle_call)

    rospy.spin()
