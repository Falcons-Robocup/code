""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 from robotScenarioBase import *

# measure shoot accuracy by various methods
#
# method A: repeatedly try to hit the goal post
#   * the robot will produce a shot from wherever it finds the ball
#   * standard motionPlanning::shootAtTarget is used
#   * optionally a fixed (x,y) coordinate can be given to move towards first, before shooting
#   * success rate has to be determined manually: was the goal post hit, or missed (or half)
#


# settings
GOAL_POST_X = 1.06 # true for Locht goal, assuming it is placed symmetrically
GOAL_POST_Y = 9
SHOOT_POSITIONS = [6, 5, 4, 3]

def _singleShot(x, y, x0, y0, callback):    
    while True:
        # wait until we see the ball
        while not robot.seeBall():
            sleep(1)
        # get ball
        robot.getBall()
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to target position
        if y0 != None:
            x0 = float(x0)
            y0 = float(y0)
            phi = robot.calculateTargetPhi(x0, y0)
            move(x0, y0, phi)
        # check ball possession
        if not robot.hasBall():
            continue
        # optionally: move to shooting position
        if y != None:
            x = float(x)
            y = float(y)
            phi = robot.calculateTargetPhi(x, y)
            robot.move(x, y, phi)
        # check ball possession
        if robot.hasBall():
            break;
    # execute the behavior
    callback()
    sleep(2)

def _repeater(x, y, x0, y0, callback):
    while True:
        _singleShot(x, y, x0, y0, callback)


def shootAccuracyPhi(fromX=None, fromY=None, x0=None, y0=None, goalX=GOAL_POST_X, goalY=GOAL_POST_Y):
    """
    Repeatedly shoot at the goalpost.
    Optionally, drive towards given position (x,y) to shoot from.
    Optionally, drive via position (x0,y0), to achieve systematic rotation with ball.
    """
    def executor():
        # aim and shoot via motionPlanning::shootAtTarget (blocking tst service)
        robot.shootAt(float(goalX), float(goalY))
    _repeater(fromX, fromY, x0, y0, executor)


def shootAccuracyZ(fromX=None, fromY=None, x0=None, y0=None, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    Repeatedly perform a lobshot at the middle of goal bar.
    Optionally, drive towards given position (x,y) to shoot from.
    Optionally, drive via position (x0,y0), to achieve systematic rotation with ball.
    """
    def executor():
        # aim and shoot via motionPlanning (blocking tst service)
        robot.lobShotAt(float(goalX), float(goalY), float(z))
    _repeater(fromX, fromY, x0, y0, executor)

def straightShotAccuracy(fromY=SHOOT_POSITIONS, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    from given positions, perform a straight shot  at the middle of goal bar.
    """
    def executor():
        # aim and shoot via motionPlanning::shootAtTarget (blocking tst service)
        robot.shootAt(float(goalX), float(goalY), float(z))
    
    for y in fromY:
        # shoot 3 times from the same position
        for _ in range(3):
            _singleShot(0, y, None, None, executor)

def lobShotAccuracy(fromY=SHOOT_POSITIONS, goalX=0, goalY=GOAL_POST_Y, z=1.05):
    """
    from given positions, perform a lobshot  at the middle of goal bar.
    """
    def executor():
        # aim and shoot via motionPlanning (blocking tst service)
        robot.lobShotAt(float(goalX), float(goalY), float(z))
    
    for y in fromY:
        # shoot 3 times from the same position
        for _ in range(3):
            _singleShot(0, y, None, None, executor)
