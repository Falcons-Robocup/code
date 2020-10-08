""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 from robotScenarioBase import *
import math


# Script for system demo: keeper
# TODO: migrate to blockly
# See wiki for setup, scoring, etc.




# configuration
REPEATS        = 2 # number of times shooting left and right per distance
FIELD_SIZE_Y   = 9 # field size configuration, TODO use environmentField
DISTANCES      = [3, 6, 9] # from how far to shoot, tradeoff keeper response time versus aim interpretation
GETBALL_RADIUS = 2 # set to lower
AIM_FORWARD    = math.pi * 0.5 # nominal aim forward
AIM_X_LEFT     = -0.75 # x coordinate of left shot
AIM_X_RIGHT    = 0.75 # x coordinate of right shot
AIM_Z          = 0.5 # z coordinate of all shots
AIM_Y          = FIELD_SIZE_Y # y coordinate of all shots
AIM_TOLERANCE  = 0.01
FLYIN_OFFSET   = 0.3 # how far to drive back
FLYIN_SPEED    = 0.3 # how fast to drive back


# helper function
def _performShot(robotPos, shootTarget):
    while not robot.hasBall():
        # get the ball
        if robot.ballCloseBy(robotPos[0], robotPos[1], GETBALL_RADIUS):
            robot.getBall()
        sleep(1)
    # to minimize sensitivity for shooting aim problems,
    # we perform a little fly-in move, or actually: a small step-back step-forward
    robot.move(*robotPos)
    robotPosFineAim = (robotPos[0], robotPos[1], robot.calculateTargetPhi(shootTarget[0], shootTarget[1]))
    robot.move(*robotPosFineAim, phiTol=AIM_TOLERANCE)
    # move backwards using speed, otherwise robot might start to rotate (Tokyo Drift),
    # which might negatively affect shooting accuracy, rendering this keeper evaluation useless ..
    robot.setVelocity(0.0, -FLYIN_SPEED, 0.0, FLYIN_OFFSET / FLYIN_SPEED)
    robot.move(*robotPosFineAim, phiTol=AIM_TOLERANCE)
    # now any ballHandling arm errors should be minimized
    robot.shootAt(*shootTarget)
    sleep(1)


def _shootAtKeeperSequence():
    # main sequence for the attacker
    for dist in DISTANCES:
        pos = (0, FIELD_SIZE_Y - dist, AIM_FORWARD)
        for it in range(REPEATS):
            # shoot left and right from this position
            _performShot(pos, (AIM_X_LEFT, AIM_Y, AIM_Z))
            _performShot(pos, (AIM_X_RIGHT, AIM_Y, AIM_Z))


def keeperDemo():
    if robot.myRobotId() == 1:
        # TODO: automatically configure on isolated worldModelSync
        robot.behavior("B_GOALKEEPER") # blocks forever
    else:
        _shootAtKeeperSequence()

