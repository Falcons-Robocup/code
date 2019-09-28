""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from interceptBall import zoneInterceptBall


def dispatchBall(shootPower=40, homeX=0, homeY=-6, homePhi=0.5*pi):
    """
    Supporting scenario for kpiFetchBall.
    A robot is shooting the ball away from a fixed position with fixed speed, so other robot can chase it.
    After shooting, it will intercept and reposition.
    """
    # sanitize inputs
    shootPower = float(shootPower)
    homeX = float(homeX)
    homeY = float(homeY)
    homePhi = float(homePhi)
    myPos = (homeX, homeY, homePhi)
    xyTol = 0.2 
    phiTol = 0.05
    dt = 0.1
    # iterate
    while True:
        trace("dispatch iteration")
        sleep(dt)
        # intercept with current zone or get close ball 
        r = zoneInterceptBall(*myPos, radius=2.0)
        log("intercept finished: " + r, 0)
        # check if we (still) have the ball - we might have lost it
        if hasBall():
            # move back to home position
            move(*myPos, xyTol=xyTol, phiTol=99) # blocking
            log("shooting the ball", 0)
            if hasBall():
                kick(shootPower)
                sleep(2)

