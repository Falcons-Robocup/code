# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3


import math
import copy

def project_angle_0_2pi(angle):
    while (angle < 0):
        angle += 2*math.pi
    while (angle > 2*math.pi):
        angle -= 2*math.pi
    return angle

def project_angle_mpi_pi(angle):
    while (angle < -math.pi):
        angle += 2*math.pi
    while (angle > math.pi):
        angle -= 2*math.pi
    return angle

def angle_between_two_points_0_2pi(x1, y1, x2, y2):
    angle = math.atan2(y2 - y1, x2 - x1)
    return project_angle_0_2pi(angle)

class Vec2d():
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __mul__(self, f):
        return Vec2d(self.x * f, self.y * f)

    def __add__(self, other):
        return Vec2d(self.x + other.x, self.y + other.y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        return Vec2d(self.x - other.x, self.y - other.y)

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

    def __repr__(self):
        return "(%6.2f, %6.2f)" % (self.x, self.y)


class Vec3d():
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __mul__(self, f):
        return Vec3d(self.x * f, self.y * f)

    def __add__(self, other):
        return Vec3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __sub__(self, other):
        return Vec3d(self.x - other.x, self.y - other.y, self.z - other.z)

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    def size(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __str__(self):
        return "(%6.2f, %6.2f, %6.2f)" % (self.x, self.y, self.z)


class RobotPose():
    def __init__(self, x=0.0, y=0.0, Rz=0.0):
        self.x = float(x)
        self.y = float(y)
        self.Rz = float(Rz)

    def xy(self):
        return Vec2d(self.x, self.y)

    def __add__(self, other):
        return RobotPose(self.x + other.x, self.y + other.y, project_angle_0_2pi(self.Rz + other.Rz))

    def __sub__(self, other):
        return RobotPose(self.x - other.x, self.y - other.y, project_angle_mpi_pi(self.Rz - other.Rz))

    def __iadd__(self, other):
        self.x = other.x + self.x
        self.y = other.y + self.y
        self.Rz = project_angle_0_2pi(other.Rz + self.Rz)
        return self

    def __mul__(self, f):
        return RobotPose(self.x * f, self.y * f, project_angle_0_2pi(self.Rz * f))

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.Rz = project_angle_0_2pi(-self.Rz)

    def __repr__(self):
        Rz = project_angle_0_2pi(self.Rz)
        return "(%7.3f, %7.3f, %7.3f)" % (self.x, self.y, Rz)

    def transform_rcs2fcs(self, robotpos):
        # first rotate
        angle = -(math.pi*0.5 - robotpos.Rz)
        xyrot = self.xy().rotate(angle)
        # then translate
        self.x = xyrot.x + robotpos.x
        self.y = xyrot.y + robotpos.y
        # and set angle
        self.Rz = project_angle_0_2pi(self.Rz + angle)
        return self

    def transform_fcs2rcs(self, robotpos):
        # first translate
        xy = self.xy() - robotpos.xy()
        # then rotate
        angle = +(math.pi*0.5 - robotpos.Rz)
        xyrot = xy.rotate(angle)
        self.x = xyrot.x
        self.y = xyrot.y
        # and set angle
        self.Rz = project_angle_0_2pi(self.Rz + angle)
        return self

    def transform_fcs2acs(self, playing_left_to_right):
        if not playing_left_to_right:
            # rotate half a circle
            self.x = -self.x
            self.y = -self.y
            self.Rz = project_angle_0_2pi(self.Rz + math.pi);
        return self

    def transform_acs2fcs(self, playing_left_to_right):
        # same as fcs2acs: # rotate half a circle
        self.transform_fcs2acs(playing_left_to_right)


# velocity specialization

class RobotVelocity(RobotPose):

    # disable some operators

    def __add__(self, *args):
        raise Exception("not supported operator")

    def __sub__(self, *args):
        raise Exception("not supported operator")

    def __iadd__(self, *args):
        raise Exception("not supported operator")

    # specialize operators

    def __mul__(self, f):
        return RobotVelocity(self.x * f, self.y * f, self.Rz * f)

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.Rz = -self.Rz

    # specialize transformations

    def transform_rcs2fcs(self, robotpos):
        angle = -(math.pi*0.5 - robotpos.Rz)
        xynew = self.xy().rotate(angle)
        self.x = xynew.x
        self.y = xynew.y
        # do not update vRz
        return self

    def transform_fcs2rcs(self, robotpos):
        angle = (math.pi*0.5 - robotpos.Rz)
        xynew = self.xy().rotate(angle)
        self.x = xynew.x
        self.y = xynew.y
        # do not update vRz
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


