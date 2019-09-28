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


# SETUP part 1:
# * register this scenario as post-software-start trigger for both robots
# * make sure WIFI is disabled - for old CPU, unplug dongle, for new CPU command-line??
# 
# SETUP part 2:
# * place the robots on the same x coordinate facing the same direction
# * the ball will be passed on that x line
# * so choose x such that the obstacles are not nearby
# * roles are symmetrical, which simplifies things considerably
# * push some button to start the software (either full autoboot, or linked with inplay button)
 

def TCrun4(passRadius=3, passPower=40, timeout=300, searchY=6):
    # lock X position
    ownPos = Position2D(*ownPosition())
    x = ownPos.x
    # calculate initial search position and base targets
    initTargets = [(x, -searchY, 0.5*pi), (x, searchY, -0.5*pi)]
    passTargets = [(x, -passRadius, 0.5*pi), (x, passRadius, -0.5*pi)]
    # figure out starting side (need to be global so we can use it in flip())
    global targetIdx
    targetIdx = (int)(ownPos.y > 0)
    # helper functions
    def TCrun4_target():
        return passTargets[targetIdx]
    def TCrun4_intercept():
        zoneInterceptBall(*TCrun4_target())
    def TCrun4_flip():
        global targetIdx
        targetIdx = 1 - targetIdx
    def TCrun4_move():
        move(*TCrun4_target())
    def TCrun4_pass():
        kick(passPower)
        sleep(3) # avoid immediately chasing the ball
    # construct action sequence (to be looped)
    actionSequence = [TCrun4_intercept, TCrun4_flip, TCrun4_move, TCrun4_pass, TCrun4_flip, TCrun4_move]
    actionIdx = 0
    # move to initial target
    move(*initTargets[targetIdx])
    # determine first action
    if ballOnSameHalf():
        getBall()
        actionIdx = 2
    # iterate
    tStart = time()
    elapsed = 0
    first = True
    while elapsed < timeout:
        # perform action
        actionSequence[actionIdx]()
        # select next action
        actionIdx = (1 + actionIdx) % len(actionSequence)
        # check the time
        elapsed = time() - tStart


