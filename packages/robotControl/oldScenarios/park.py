""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



def park(parkX=-6, parkY=-8, dy=1):
    """
    After having played, return robots to park positions.
    """
    
    ## Sanitize inputs (may be strings).
    parkX = float(parkX)
    parkY = float(parkY)
    dy    = float(dy)
    deltaDistance = 2.0
    sleepTime = 0.5
    targetPhi = 0.5

    # helper function to calculate the target position based on robot number
    def parkTarget(idx):
        return Position2D(parkX, parkY + dy * idx, targetPhi)
            
    ## Calculate target. All robots on a line (x fixed).
    ar = activeRobots()
    idx = ar.index(robotId())
    myTarget = parkTarget(idx)
    
    # "hack" to avoid friendly collisions: wait until all previous robots are 
    # somewhat closer to target.
    if idx != 0:
        waiting = True
        while waiting:
            # sleep
            sleep(sleepTime)
            # calculate my distance
            myPosition = Position2D(*getPosition())
            myDistance = (myTarget - myPosition).xy().size()
            # check all previous robots
            allRobotsCloseEnough = True
            for otherRobotId in ar[0:idx]:
                otherPosition = Position2D(*getPosition(otherRobotId))
                otherTarget = parkTarget(ar.index(otherRobotId))
                otherDistance = (otherTarget - otherPosition).xy().size()
                # check if other robot is closer than this robot; use a tolerance
                if otherDistance > myDistance - deltaDistance:
                    allRobotsCloseEnough = False
            # re-determine waiting state
            waiting = not allRobotsCloseEnough
        
    ## Post: robot is ready to go to its parking position. Use 'slow' flag.
    moveToTarget(myTarget.x, myTarget.y, True)
    
