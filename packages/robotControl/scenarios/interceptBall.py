""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from robotRosInterface import *


        
def zoneInterceptBall(x, y, phi, radius=2, repositionRadius=None):
    """
    Core intercept action, which stays within specified zone, mainly stimulating the intercept action.
    If the ball is closeby, it will get it.
    This function is blocking and will only exit in the following situations, returning a string:
       SUCCESS     the ball was directly intercepted in the zone, no fallback was used
       FALLBACK    interception failed, but fallback getBall succeeded
       FAILED      the ball got close, but robot could not obtain it
    """
    # sanitize inputs
    x = float(x)
    y = float(y)
    phi = float(phi)
    radius = float(radius)
    if repositionRadius == None:
        repositionRadius = 0.5 * radius
    else:
        repositionRadius = float(repositionRadius)
    dt = 0.1
    # internal state tracking
    global lastState
    lastState = ""
    def logState(newState):
        global lastState
        if newState != lastState:
            log(newState, 0)
        lastState = newState
    # iterate
    log(">zoneInterceptBall x=%6.2f y=%6.2f phi=%6.2f radius=%6.2f" % (x, y, phi, radius))
    result = "RUNNING"
    ballWasCloseBy = False
    while True:
        # evaluate what to do
        if robotCloseBy(x, y, phi, xyTol=repositionRadius, phiTol=1.5):
            # close to designated position
            if hasBall():
                # make sure teamplay action is not continuing
                stop()
                # check for success and report
                if lastState == "intercepting":
                    result = "SUCCESS"
                else:
                    result = "FALLBACK"
                break
            else:
                # decide to chase or intercept
                # we expect that action interceptBall also faces the ball 
                logState("intercepting")
                r = action('interceptBall')
                if ballCloseBy(x, y, xyTol=radius):
                    ballWasCloseBy = True
                if r == "FAILED": 
                    while ballCloseBy(x, y, xyTol=radius) and not hasBall():
                        logState("fallback getball")
                        action('getBallOnVector') # not blocking, so outer move keeps the robot in its zone
                        sleep(dt)
                    # check for failure
                    if not ballCloseBy(x, y, xyTol=radius) and ballWasCloseBy:
                        result = "FAILED"
                        break
        else:
            # drifted away too much, coarsely reposition
            logState("repositioning")
            move(x, y, phi, xyTol=0.1) # blocking
        sleep(dt)
    # done
    log("<zoneInterceptBall " + result)
    return result
    
def interceptBall(radius=3, idx=None, N=None):
    """
    Position one or more robots in a circle, have them pass around balls.
    The core of this scenario is the interceptBall action. Around it, some re-positioning is done.
    This scenario is useful for motion- and ballTracking tuning.
    """
    # sanitize inputs
    if N != None:
        N = int(N)
    if idx != None:
        idx = int(idx)
    radius = float(radius)    
    # settings
    dt = 0.1
    center = (0,0)
    # statistics
    numSuccess = 0
    numFail = 0
    # iteration
    while True:
        sleep(dt)
        # get list of active robots
        if N == None:
            N = len(activeRobots())
        if idx == None:
            idx = myRelIndex()
        # calculate myPos and nextPos as function of number of active robots
        myPos = calcCirclePos(idx, N, radius, center)
        nextPos = calcCirclePos(1 + idx, N, radius, center)
        # evaluate what to do
        r = zoneInterceptBall(*myPos, radius=2.0)
        if r == "SUCCESS":
            numSuccess += 1
            log("success! (running total: %.1f %%)" % (100.0 * numSuccess / (1.0 * (numFail + numSuccess))), 0)
        else:
            numFail += 1
            log("failure! (running total: %.1f %%)" % (100.0 * numSuccess / (1.0 * (numFail + numSuccess))), 0)
        if hasBall():
            log("turning and shooting the ball", 0)
            # sleep(1) # should not be needed
            # turn towards next interceptor
            facePosition(nextPos[0], nextPos[1])
            if hasBall(): # check if we (still) have the ball - we might have lost it      
                actionBlocking("shoot shootType=passTowardsNearestTeammember") # using teamplay to pass
                sleep(0.5) # needed to ensure the robot does not think it still has the ball and shoots again
                # move back to home position
                move(*myPos, xyTol=0.1) # blocking

