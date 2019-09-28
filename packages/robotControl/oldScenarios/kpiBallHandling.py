""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



def kpiBhSpeedX():
    """
    Move sideways and determine the speed at which the ball goes loose.
    """
    # first move to center
    move(0, 0, 0)
    # wait until robot has the ball
    while not hasBall():
        sleep(1)
    # start the measurement sequence
    speedIncrement = 0.1
    currentSpeed = 0.5
    resultSpeed = 0
    while hasBall():
        print "trying %6.2f m/s" % (currentSpeed)
        # drive in positive X direction
        speed(currentSpeed, 0, 0, 5)
        sleep(1)
        # drive in negative X direction
        speed(-currentSpeed, 0, 0, 5)
        sleep(1)
        if hasBall():
            resultSpeed = currentSpeed
        # increase speed
        currentSpeed += speedIncrement
    # report result
    print "result: vx = %6.2f m/s" % (resultSpeed)

def kpiBhSpeedY():
    """
    Move forwards and backwards, determine the speed at which the ball goes loose.
    """
    # first move position on centre line
    move(4, 0, 1.57)
    # wait until robot has the ball
    while not hasBall():
        sleep(1)
    # start the measurement sequence
    speedIncrement = 0.1
    currentSpeed = 0.5
    resultSpeed = 0
    while hasBall():
        print "trying %6.2f m/s" % (currentSpeed)
        # drive in positive Y direction
        speed(0, currentSpeed, 0, 3)
        sleep(1)
        # drive in negative Y direction
        speed(0, -currentSpeed, 0, 3)
        sleep(1)
        if hasBall():
            resultSpeed = currentSpeed
        # increase speed
        currentSpeed += speedIncrement
    # report result
    print "result: vy = %6.2f m/s" % (resultSpeed)
    
def kpiBhSpeedPhi():
    """
    Rotate and determine the speed at which the ball goes loose.
    """
    # first move to center
    move(0, 0, 0)
    # wait until robot has the ball
    while not hasBall():
        sleep(1)
    # start the measurement sequence for positive vphi
    speedIncrement = 0.1
    currentSpeed = 1.2
    positiveResultSpeed = 0
    while hasBall():
        print "trying %6.2f rad/s" % (currentSpeed)
        # positive rotation
        speed(0, 0, currentSpeed, 5, stopAfterwards=False)
        if hasBall():
            positiveResultSpeed = currentSpeed
        # increase speed
        currentSpeed += speedIncrement
    # report result
    print "result positive vphi: vphi = %6.2f rad/s" % (positiveResultSpeed)

    # reposition to center
    move(0, 0, 0)
    # wait until robot has the ball
    while not hasBall():
        sleep(1)
    # start the measurement sequence for negative vphi
    speedIncrement = 0.1
    currentSpeed = 1.2
    negativeResultSpeed = 0
    while hasBall():
        print "trying %6.2f rad/s" % (currentSpeed)
        # negative rotation
        speed(0, 0, -currentSpeed, 5, stopAfterwards=False)
        if hasBall():
            negativeResultSpeed = currentSpeed
        # increase speed
        currentSpeed += speedIncrement
    # report result
    print "result negative vphi: vphi = %6.2f rad/s" % (negativeResultSpeed)
    print "result positive vphi = %6.2f rad/s, negative vphi = %6.2f rad/s" % (positiveResultSpeed, negativeResultSpeed)

def kpiBhAccPhi(vPhi=2.0): #Default = 2.0 rad/s
    """
    Rotate and determine the acceleration at which the ball goes loose.
    """
    # first move to center
    move(0, 0, 0)
    # wait until robot has the ball
    while not hasBall():
        sleep(1)
    # Set the pathplanning phi maximum velocity to given rad/s
    setConfig("PathPlanningNode", {"normal_Limiters_maxVelPhi": vPhi})

    # start the measurement sequence for different accelerations
    accIncrement = 0.5
    currentAcc = 3.0
    resultAcc = 0
    while hasBall():
        print "trying %6.2f rad/s" % (currentAcc)
        setConfig("PathPlanningNode", {"normal_Limiters_maxAccPhi": currentAcc})
        # Turn around.
        move(0, 0, 3.14)
        # Again.
        move(0, 0, 0)
        if hasBall():
            resultAcc = currentAcc
        # increase acceleration
        currentAcc += accIncrement
    # report result
    print "result phi acceleration: aphi = %6.2f rad/s" % (resultAcc)

