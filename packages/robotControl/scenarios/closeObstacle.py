""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



def closeObstacle(homeX=0, homeY=0, homePhi=0):
    """
    Test if robot is able to avoid close obstacle, while not being too afraid to get closeby ball.
    
    Require 3 robots.
    * r1 is support, used only to spot the ball, so it does not run this or any other scenario
    * r2 is offline, as the 'enemy' obstacle (give him the anti-collision band)
    * r3 is the active moving robot, which runs this scenario

    The sequence for r3 is as follows:
    * move to home position
    * getBall
    * release the ball
    """

    # sanitize inputs
    homeX = float(homeX)
    homeY = float(homeY)
    homePhi = float(homePhi)
    
    # iterate
    while True:
        # move to home position
        move(homeX, homeY, homePhi)
        # get ball
        enableBallHandlers()
        while not hasBall(robotId()):
            actionBlocking("getBallOnVector")
        tpControl("disabled")
        # release ball, back away
        disableBallHandlers()
        speed(0, -0.4, 0, 1)
        # wait
        sleep(2)

