""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import math
import rospy


THETA_BIAS=math.pi/2   #correct THETA such that 0 is as viewing towards opponentside

def theta( angle ): #input angle in degrees in string format, ready to be inserted into parameters
   corrected=angle*math.pi/180 + THETA_BIAS
   return "{:.2f}".format(corrected)  #returns corrected in radians

def distanceCalc( x1,y1, x2,y2):   #calc distance and angle between points where angle 0 is looking Y+
   dx = float(x2) - float(x1)
   dy = float(y2) - float(y1)
   hypothenusa= math.sqrt( dx**2 + dy**2 )
   #rads = math.atan2(-dy,dx)
   rads = math.atan2(-dx,dy)  #correction, looking to Y+ = angle 0
   rads %= 2*math.pi
   degs = math.degrees(rads)
   #print "Rads=%.1f  Degs=%.1f" %( rads, degs )
   return ( hypothenusa , degs )

def waitForService( serviceName ):
    print("%s, waiting..." % serviceName),
    rospy.wait_for_service( serviceName )
    print " OK"


