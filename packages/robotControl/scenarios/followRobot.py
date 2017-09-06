""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



def followRobot(targetId=None, distance=1):
    """
    Follow specified robot at a fixed distance.
    If no robot specified, then the one with lower index is chosen.
    Default follow position is 1 meter behind the robot, facing it.
    """
    # select robot to follow
    if targetId == None:
        a = activeRobots()
        idx = a.index(robotId())
        if idx == 0:
            raise Exception("cannot choose a robot to follow")
        targetId = a[idx-1]
    # sanitize inputs
    targetId = int(targetId)
    distance = float(distance)
    # execute
    while True:
        # calculate target position
        targetRcs = Position2D(0, -distance, 0.5*pi)
        posOtherFcs = Position2D(*getPosition(targetId))
        targetFcs = targetRcs.transform_rcs2fcs(posOtherFcs)
        # publish
        setTarget(targetFcs.x, targetFcs.y, targetFcs.phi)
        sleep(0.01)


