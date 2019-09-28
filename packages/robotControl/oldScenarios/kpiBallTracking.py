""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotInterface import *
from robotActions import *
from monitorBall import monitorBall



def kpiStaticBallStability(radius=3.0, vel=1.0, idx=None, N=None):
    """
    Check how robot(s) perceive a stationary ball. 
    Requires the ball to be placed manually at (0,0).
    The sequence starts with stationary robot, then it will move in a circle around the ball while facing it.
    Multiple robots can be used simultaneously, provided that the start is in sync.
    """
    # sanitize inputs
    if N != None:
        N = int(N)
    else:
        N = len(activeRobots())
    if idx != None:
        idx = int(idx)
    else:
        idx = myRelIndex()
    radius = float(radius)
    vel = float(vel)
    # settings
    center = (0,0)
    xyTol = 0.03 
    phiTol = 0.025
    # FINE move to initial position
    myPos = calcCirclePos(idx, N, radius, center)
    log('moving to initial position...', 0)
    move(*myPos, xyTol=xyTol, phiTol=phiTol) # blocking
    # measure stability while robot is not moving
    log('measuring ball stability while robot is not moving...', 0)
    timeout = 10
    startThread(monitorBall, timeout)
    sleep(timeout + 2) # sleep a bit longer
    # measure stability while robot is moving away from and towards the ball
    myPos = calcCirclePos(idx, N, 1, center)
    move(*myPos, xyTol=xyTol, phiTol=phiTol) # blocking
    log('measuring ball stability while robot is moving towards and away from the ball...', 0)
    distance = radius - 1
    timeout = distance / vel
    startThread(monitorBall, 2*timeout)
    speed(0.0, -vel, 0, timeout)
    speed(0.0, +vel, 0, timeout)
    sleep(5)
    # measure stability while robot is moving in a circle around the ball
    myPos = calcCirclePos(idx, N, radius, center)
    move(*myPos, xyTol=xyTol, phiTol=phiTol) # blocking
    log('measuring ball stability while robot is moving around the ball...', 0)
    timeout = 10
    startThread(monitorBall, timeout)
    speed(vel, 0.0, vel / radius, timeout + 2)
    # done

