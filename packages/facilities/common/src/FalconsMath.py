# Copyright 2015-2020 Tim Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3


import math

TWO_PI=math.pi * 2

def distanceCalc( x1, y1, x2, y2):   #calc distance and angle between points where angle 0 is looking Y+
    dx = float(x2) - float(x1)
    dy = float(y2) - float(y1)
    hypothenusa= math.sqrt( dx**2 + dy**2 )
    #rads = math.atan2(-dy,dx)
    rads = math.atan2(-dx,dy)  #correction, looking to Y+ = angle 0
    rads %= 2*math.pi
    degs = math.degrees(rads)
    #print "Rads=%.1f  Degs=%.1f" %( rads, degs )
    return [ hypothenusa , degs ]


def phidiff(phi1, phi2):
    """
    Calculate angle difference, so it can be checked on size. 
    Similar to 'project_angle_mpi_pi' in cFalconsCommon.cpp.
    """
    dphi = phi1 - phi2
    while dphi < -math.pi:
        dphi += 2*math.pi
    while dphi > math.pi:
        dphi -= 2*math.pi
    return dphi

def normalizeAngle( theta ):
   # Normalzie theta in radians to [0, 2*pi)
   return theta % TWO_PI

