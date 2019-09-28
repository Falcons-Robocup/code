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




import math

# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


class Pose():

    def __init__(self, x=0.0, y=0.0, phi=0.0):
        self.x = x
        self.y = y
        self.phi = phi

    def __add__(self, other):
        return Pose(self.x + other.x, self.y + other.y, project_angle_0_2pi(self.phi + other.phi))
        
    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, project_angle_0_2pi(self.phi - other.phi))
        
    def __iadd__(self, other):
        self.x = other.x + self.x
        self.y = other.y + self.y
        self.phi = project_angle_0_2pi(other.phi + self.phi)
        return self
        
    def __mul__(self, f):
        return Pose(self.x * f, self.y * f, project_angle_0_2pi(self.phi * f))

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.phi = project_angle_0_2pi(-self.phi)

    def __str__(self):
        self.phi = project_angle_0_2pi(self.phi)
        return "(%6.2f,%6.2f,%6.2f)" % (self.x, self.y, self.phi)

    def __repr__(self):
        return self.__str__()
        
    def r(self):
        return math.sqrt(self.x**2 + self.y**2)

    def aimingAt(self, pose, angleThreshold=0.1):
        angle = -math.atan2(pose.x - self.x, pose.y - self.y)
        trace("dphi=%.2f", project_angle_mpi_pi(angle - self.phi))
        return abs(project_angle_mpi_pi(angle - self.phi)) < angleThreshold

    def aimingAtGoal(self, dxThreshold=1.0):
        result = False
        goalY = 9
        goalRadius = 1
        if self.y < 9:
            x = goalRadius + dxThreshold
            angleLeft = -math.atan2(-x - self.x, goalY - self.y)
            angleRight = -math.atan2(x - self.x, goalY - self.y)
            result = (project_angle_mpi_pi(self.phi - angleLeft) < 0) and (project_angle_mpi_pi(self.phi - angleRight) > 0)
        return result

    def insideKeeperArea(self):
        return (self.y > -10) and (self.y < -7) and (abs(self.x) < 2)
        
    def onField(self, dist=1.0):
        return (abs(self.x) < 6 + dist) and (abs(self.y) < 9 + dist)


def project_angle_0_2pi(angle):
	while (angle < 0):
	    angle += 2*math.pi
	while (angle > 2*math.pi):
	    angle -= 2*math.pi
	return angle


def project_angle_mpi_pi(angle):
	while (angle < math.pi):
	    angle += 2*math.pi
	while (angle > math.pi):
	    angle -= 2*math.pi
	return angle


# autotester
if __name__ == '__main__':
    # test Pose.aimingAt
    p = Pose(0, 0, 0)
    for y in [-1, 0, 1]:
        for x in [-1, 0, 1]:
            if x != 0 or y != 0:
                expected = (x == 0) and (y == 1)
                assert(p.aimingAt(Pose(x, y)) == expected)
    p = Pose(4, 4, 3*math.pi/4)
    for y in [3, 4, 5]:
        for x in [3, 4, 5]:
            if x != 4 or y != 4:
                expected = (x == 3) and (y == 3)
                assert(p.aimingAt(Pose(x, y)) == expected)
    assert(Pose(0, 0, 1.57).aimingAt(Pose(-2, 0))) # kickoff
    assert(Pose(0, 0, -1.57).aimingAt(Pose(2, 0))) # kickoff
    # test Pose.aimingAtGoal
    assert(Pose(0, 0, 0).aimingAtGoal())
    assert(Pose(1.5, 0, 0).aimingAtGoal())
    assert(Pose(-1.5, 0, 0).aimingAtGoal())
    assert(False == Pose(2.5, 0, 0).aimingAtGoal())
    assert(False == Pose(-2.5, 0, 0).aimingAtGoal())
    assert(Pose(3, 6, 0.7).aimingAtGoal())
    
    

