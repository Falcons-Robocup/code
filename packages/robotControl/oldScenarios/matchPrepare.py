""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from interceptBall import *



def matchPrepare(waitXThreshold=5, doRotation=1, doPassAround=1):
    """
    Move to starting positions, can be from anywhere.
    Starting positions are chosen such that the move to kickoffPrepare position has little chance to 
    lead to friendly collisions.
    
    If moving away from park positions, then robot will wait for previous robot to have moved away a bit,
    to avoid friendly collisions.
    
    Optionally, a test sequence is executed:
    * 1 full rotation (to check that all wheel motors are equally contributing)
    * pass around the ball while looking at it
      (to check motion tuning, ball possession and worldModel ballTracking)
    """
    
    log("preparing for match", 0)
    ## Sanitize inputs (may be strings).
    waitXThreshold = float(waitXThreshold)
    doRotation = bool(doRotation)
    doPassAround = bool(doPassAround)
    
    ## Determine my starting position.
    ar = activeRobots()
    idx = ar.index(robotId())
    targetPhi = 0.5 * pi
    targets = [ Position2D( 0.0, -6.5, targetPhi), \
                Position2D(-4.0, -5.0, targetPhi), \
                Position2D( 3.0, -6.0, targetPhi), \
                Position2D(-3.0, -1.0, targetPhi), \
                Position2D( 4.0, -2.0, targetPhi), \
                Position2D( 5.0, -4.0, targetPhi) ]
    myTarget = targets[robotId()-1]
    
    ## First robot does not have to avoid anything; don't wait.
    if idx != 0:
        ## "Hack" to avoid friendly collisions when moving all at the same time from park position: 
        # wait until previous robot has moved away a bit.
        sleepTime = 0.25
        while True:    
            otherRobotId = ar[idx -1]
            otherPosition = Position2D(*getPosition(otherRobotId))
            if abs(otherPosition.x) < waitXThreshold:
                break
            sleep(sleepTime)
    
    # move to starting position with motionProfile "SETPIECE"
    move(myTarget.x, myTarget.y, myTarget.phi, motionProfile=1)
    
    # motor check: one full rotation
    if doRotation:
        rotationStart = Position2D(*getPosition())
        vPhi = 1.337
        t = 2.0 * pi / vPhi
        speed(0, 0, vPhi, t)
        # position check
        if not robotCloseBy(rotationStart.x, rotationStart.y, xyTol=0.1):
            log("robot has drifted a bit - check motors", level=0, eventType=1)
        
    # pass around the ball
    # from r2 to r3 to r4 etc
    # first define helper functions
    def waitToStartIntercept():
        sleepTime = 0.25
        if idx <= 1:
            return # lowest robot can start immediately (ignore keeper)
        otherRobotId = ar[idx-1]
        while not hasBall(otherRobotId):    
            sleep(sleepTime)
    def passToNextRobot():
        # determine where to pass the ball to
        if idx == len(ar)-1:
            # last robot in sequence passes towards 0,0
            passTarget = (0,0)
        else:
            # any other robot passes towards next robot
            # note that this requires the robots to be positioned in such a way that this works 
            passTarget = getPosition(ar[idx+1]) 
        # TODO rather use a teamplay action, but unfortunately there is no passTowardsTarget...
        facePosition(passTarget[0], passTarget[1])
        if hasBall(): # check if we (still) have the ball - we might have lost it      
            #actionBlocking("shoot shootType=passTowardsNearestTeammember") # TODO cannot use this
            kick()
    # now execute the pass-around sequence (all robots except keeper)
    if doPassAround and (idx > 0):
        # wait until previous robot has obtained the ball
        waitToStartIntercept()
        # rotate towards ball 
        ballPos = ballPosition()
        facePosition(ballPos[0], ballPos[1])
        # intercept until the ball is received
        # (do not start intercepting earlier because then robot could respond to a ball meant for other robot)
        currPos = getPosition()
        zoneInterceptBall(currPos[0], currPos[1], currPos[2])
        # pass to the next robot
        sleep(1)
        passToNextRobot()
        # move back to starting position with motionProfile "SETPIECE"
        move(myTarget.x, myTarget.y, myTarget.phi, motionProfile=1)

    log("ready to play", 0)

