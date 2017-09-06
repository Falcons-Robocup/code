""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python


import math
import copy


def project_angle_0_2pi(angle):
	while (angle < 0):
	    angle += 2*math.pi
	while (angle > 2*math.pi):
	    angle -= 2*math.pi
	return angle

def project_angle_mpi_pi(angle):
	while (angle < math.pi):
	    angle += math.pi
	while (angle > math.pi):
	    angle -= math.pi
	return angle


class Vector2D():
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
        
    def __mul__(self, f):
        return Vector2D(self.x * f, self.y * f)
    
    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def size(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
        
    def rotate(self, angle):
        s = math.sin(angle)
        c = math.cos(angle)
        nx = c * self.x - s * self.y;
        ny = s * self.x + c * self.y;
        self.x = nx
        self.y = ny
        return self

    def __str__(self):
        return "(%6.2f, %6.2f)" % (self.x, self.y)

class Position2D():
    def __init__(self, x=0.0, y=0.0, phi=0.0):
        self.x = x
        self.y = y
        self.phi = phi

    def xy(self):
        return Vector2D(self.x, self.y)
        
    def __add__(self, other):
        return Position2D(self.x + other.x, self.y + other.y, project_angle_0_2pi(self.phi + other.phi))
        
    def __sub__(self, other):
        return Position2D(self.x - other.x, self.y - other.y, project_angle_0_2pi(self.phi - other.phi))
        
    def __iadd__(self, other):
        self.x = other.x + self.x
        self.y = other.y + self.y
        self.phi = project_angle_0_2pi(other.phi + self.phi)
        return self
        
    def __mul__(self, f):
        return Position2D(self.x * f, self.y * f, project_angle_0_2pi(self.phi * f))

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.phi = project_angle_0_2pi(-self.phi)

    def __str__(self):
        self.phi = project_angle_0_2pi(self.phi)
        return "pos=(%6.2f, %6.2f, %6.2f)" % (self.x, self.y, self.phi)

    def transform_rcs2fcs(self, robotpos):
        # first rotate
        angle = -(math.pi*0.5 - robotpos.phi)
        xyrot = self.xy().rotate(angle)
        # then translate
        self.x = xyrot.x + robotpos.x
        self.y = xyrot.y + robotpos.y
        # and set angle
        self.phi = project_angle_0_2pi(self.phi + angle)
        return self
        
    def transform_fcs2rcs(self, robotpos):
        # first translate
        xy = self.xy() - robotpos.xy()
        # then rotate
        angle = +(math.pi*0.5 - robotpos.phi)
        xyrot = xy.rotate(angle)
        self.x = xyrot.x
        self.y = xyrot.y
        # and set angle
        self.phi = project_angle_0_2pi(self.phi + angle)
        return self
        
    def transform_fcs2acs(self, playing_left_to_right):        
        if not playing_left_to_right:
            # rotate half a circle
            self.x = -self.x
            self.y = -self.y
            self.phi = project_angle_0_2pi(self.phi + math.pi);
        return self

    def transform_acs2fcs(self, playing_left_to_right):        
        # same as fcs2acs: # rotate half a circle
        self.transform_fcs2acs(playing_left_to_right)
        
    

class Velocity2D(Position2D):

    def __str__(self):
        return "vel=(%6.2f, %6.2f, %6.2f)" % (self.x, self.y, self.phi)

    def transform_rcs2fcs(self, robotpos):
        angle = -(math.pi*0.5 - robotpos.phi)
        xynew = self.xy().rotate(angle)
        self.x = xynew.x
        self.y = xynew.y
        # do not update vphi
        return self
        
    def transform_fcs2rcs(self, robotpos):
        angle = (math.pi*0.5 - robotpos.phi)
        xynew = self.xy().rotate(angle)
        self.x = xynew.x
        self.y = xynew.y
        # do not update vphi
        return self
        
    def transform_fcs2acs(self, playing_left_to_right):        
        if not playing_left_to_right:
            # rotate half a circle, xy only
            self.x = -self.x
            self.y = -self.y
        return self

    def transform_acs2fcs(self, playing_left_to_right):        
        # same as fcs2acs: # rotate half a circle, xy only
        self.transform_fcs2acs(playing_left_to_right)
        
        
# selftest
if __name__ == "__main__":
    print Position2D()
    p = Position2D(2.0, -3.1)
    print p
    q = Position2D(0, 0, 6.9) + p
    print q
    v = Velocity2D(2.0, -1.0)
    pnew = p + v.transform_rcs2fcs(p)
    print pnew # should be (  1.0,  -5.1)
    
    
