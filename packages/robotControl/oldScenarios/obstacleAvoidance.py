""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from wayPoints import*

def obstacleAvoidance(idx=None, N=None):
    # scenario to test obstacle avoidance. N=1 will have no obstacle avoidance (must be the lowest number on the field) and will drive parralell to the centre line. N=2 will have obstcale avoidance enabled and will move perpendicular to the centre line.
    # sanitize inputs
    if N != None:
        N = int(N)
    if idx != None:
        idx = int(idx)
        
    # Configure idx=1
    if idx == 1:
        setConfig("PathPlanningNode", {"Obstacle_avoidance_enabled":False}) # Set obstacle avoidance too off
        # Set waypoints (x, y, phi)
        WP1 = (-3, 5, 0)
        WP2 = (3, 5, 0)
        WP3 = (3, 5, 3.14)
        WP4 = (-3, 5, 3.14)
        wayPoints(WP1[0], WP1[1], WP1[2],WP2[0], WP2[1], WP2[2], WP3[0], WP3[1], WP3[2], WP4[0], WP4[1], WP4[2])
    
    # Configure idx =2
    if idx == 2:
        # Set waypoints (x, y, phi)
        WP1 = (0, 8, 4.71)
        WP2 = (0, 3, 4.71)
        WP3 = (0, 3, 1.57)
        WP4 = (0, 8, 1.57)
        wayPoints(WP1[0], WP1[1], WP1[2],WP2[0], WP2[1], WP2[2], WP3[0], WP3[1], WP3[2], WP4[0], WP4[1], WP4[2])
