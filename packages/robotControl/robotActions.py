""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python library for commands, on-robot.
# This file implements logic on top of the low-level ROS interface.
#
# Some of these (mostly blocking) wrappers could be relocated as test interfaces within their respective module.
# For instance, it would be useful if pathPlanning would provide blocking services for move and tokyoDrift.
# Also, teamplay could offer blocking services for an action or behavior.
# 
# Jan Feitsma, 2017-01-14



# import ROS interface
from robotRosInterface import *



def stop():
    """
    Reset the state of all software modules.
    """
    log(">stop")
    # we work top-down - first disable teamplay
    tpControl("disabled") # make sure teamplay is reset afterwards, otherwise it will not respond to refbox anymore
    # TODO clear target / disable pathPlanning?
    # finally, clear robotSpeed / brake if moving
    speed(0, 0, 0, 0)
    log("<stop")

def speed(vx, vy, vphi, t, stopAfterwards=True):
    """
    Continuously send a speed setpoint (RCS) to peripheralInterface.
    """
    log(">speed vx=%6.2f vy=%6.2f vphi=%6.2f t=%6.2f" % (vx, vy, vphi, t))
    tStart = time()
    dt = 0.05
    elapsed = 0
    while elapsed < t:
        setSpeed(vx, vy, vphi)
        sleep(dt)
        elapsed = time() - tStart
    if stopAfterwards:
        # stop
        setSpeed(0, 0, 0)
        sleep(dt)
    elapsed = time() - tStart
    log("<speed elapsed=%6.2f" % (elapsed))

def move(x, y, phi=0, xyTol=None, phiTol=None, motionProfile=None):
    """
    Basic move by feeding a target to pathPlanning. This function blocks!
    """
    log(">move x=%6.2f y=%6.2f phi=%6.2f" % (x, y, phi))
    stop() # make sure teamplay is not interfering
    tStart = time()

    # If the (XY|Phi)Tolerance is not None, dynamic_reconfigure PathPlanning to given values.
    if xyTol != None:
        setConfig("PathPlanningNode", {"normal_Limiters_tolerationXY": xyTol})
    if phiTol != None:
        setConfig("PathPlanningNode", {"normal_Limiters_tolerationPhi": phiTol})

    if motionProfile != None:
        setTargetBlocking(x, y, phi, motionProfile=motionProfile)
    else:
        setTargetBlocking(x, y, phi)
        
    speed(0, 0, 0, 0) # prevent pathPlanning drift

    elapsed = time() - tStart

    restoreConfig("PathPlanningNode")

    log("<move elapsed=%6.2f" % (elapsed))

def tokyoDrift(x, y, vPhi=3.0, phiTol=5e-2):
    """
    This function is OBSOLETE - pathPlanning now performs the maneuver automatically.
    
    Perform the 'Tokyo drift' maneuver: rotate around the ball while keeping it engaged, until facing target (x,y).
    We cannot use pathPlanning in its current form due to tight limits on rotational velocity and accelleration
    so instead we use a direct speed setpoint and a little control loop.
    """
    log(">tokyoDrift")
    dt = 0.01
    targetPhi = calculateTargetPhi(x, y)
    deltaPhi = calculateDeltaPhi(targetPhi)
    radius = 0.20 # chosen such that the ball remains in its place
    cutoff = 0.30 # TODO this is a bit magic...
    log("deltaPhi=%6.2f" % (deltaPhi))
    # rotate/drift until done
    while abs(deltaPhi) > phiTol:
        direction = 1.0 * sign(deltaPhi)
        # limit speed by linearized setpoint to prevent overshoot / instability
        vTmp = vPhi * direction
        if abs(deltaPhi) < vPhi * cutoff:
            log("limiting vTmp=%6.2f" % (vTmp))
            vTmp = deltaPhi
        # set the speed setpoint
        speed(vTmp * radius, 0, vTmp, dt, stopAfterwards=False)
        # recalculate angles
        targetPhi = calculateTargetPhi(x, y)
        deltaPhi = calculateDeltaPhi(targetPhi)
        log("deltaPhi=%6.2f" % (deltaPhi))
    # finish by braking
    setSpeed(0, 0, 0)
    sleep(dt)
    log("<tokyoDrift")

def facePosition(x, y):
    targetPhi = calculateTargetPhi(x, y)
    (robotX, robotY, robotPhi) = getPosition()
    move(robotX, robotY, targetPhi)

def isFacingPosition(x, y, phiTol=3e-2):
    """
    Return if robot is facing position.
    """
    targetPhi = calculateTargetPhi(x, y)
    (robotX, robotY, robotPhi) = getPosition()
    return robotCloseBy(robotX, robotY, targetPhi, xyTol=1.0, phiTol=phiTol)

def action(*args):
    """
    Perform an action by setting teamplay in the desired state.
    This function is not blocking, so it does not care about the result or duration of the action.
    See also: actionBlocking.
    """
    log(">action '%s'" % (' '.join(args)))
    r = tpControl("action " + ' '.join(args))
    log("<action result=" + r)
    return r

def actionBlocking(*args):
    log(">actionBlocking '%s'" % (' '.join(args)))
    tStart = time()
    actionString = "action " + ' '.join(args)
    response = ""
    dt = 0.1
    while True:
        response = tpControl(actionString)
        if response in ["FAILED", "PASSED"]:
            break
        sleep(dt)
    elapsed = time() - tStart
    log("<actionBlocking elapsed=%6.2f" % (elapsed))

def behavior(name):    
    tpControl("behavior " + name)

def behaviorBlocking(*args):
    log(">behaviorBlocking '%s'" % (' '.join(args)))
    tStart = time()
    behaviorString = "behavior " + ' '.join(args)
    response = ""
    dt = 0.1
    while True:
        response = tpControl(behaviorString)
        if response in ["FAILED", "PASSED"]:
            break
        sleep(dt)
    elapsed = time() - tStart
    log("<behaviorBlocking elapsed=%6.2f" % (elapsed))

def calcCirclePos(robotIdx, numRobots, radius=3, center=(0,0)):
    """
    Helper function to distribute robot positions on a circle.
    """
    gamma = 2*pi / numRobots
    x = radius * cos(gamma * robotIdx) + center[0]
    y = radius * sin(gamma * robotIdx) + center[1]
    phi = gamma * robotIdx - pi
    return (x, y, phi)



