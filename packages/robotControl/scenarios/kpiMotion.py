""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
import threading

noiseMargin = 0.02 #2cm

def monitorLocation():
    # Monitor the location of the robot.
    # If the location exceeds the global vars (max|min)(X|Y), this is printed on the screen.
    global keepMonitoring, monitorX, monitorY, minX, maxX, minY, maxY, violation

    while keepMonitoring:
        pos = ownPosition()

        if monitorX:
            if pos[0] < (minX - noiseMargin):
                violation = "Robot position X (%8.4f) exceeded limit (%8.4f)" % (pos[0], minX)
            elif pos[0] > (maxX + noiseMargin):
                violation = "Robot position X (%8.4f) exceeded limit (%8.4f)" % (pos[0], maxX)

        if monitorY:
            if pos[1] < (minY - noiseMargin):
                violation = "Robot position Y (%8.4f) exceeded limit (%8.4f)" % (pos[1], minY)
            elif pos[1] > (maxY + noiseMargin):
                violation = "Robot position Y (%8.4f) exceeded limit (%8.4f)" % (pos[1], maxY)


# Measure undershoot and overshoot.
# To measure undershoot, look at the final location before moving to the next action
# To measure overshoot, constantly measure the position to see if the robot overshoots the target position
def kpiMotionShortStroke():
    """
    Turn off vision and test the short stroke behavior of motion based on encoder feedback.
    """
    global keepMonitoring, monitorX, monitorY, minX, maxX, minY, maxY, violation

    # first move to center
    move(0, 0, 1.67)
    # Turn off vision
    setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0})
    setConfig("PathPlanningNode", {"normal_Limiters_tolerationXY": 0.0})

    # Define the range and step size of the short stroke steps
    start = 0.05 #5cm
    end = 0.50 #50cm
    step = 0.05 #5cm
    current = start # initialize with the start distance

    monitorX = False
    monitorY = False
    violation = None

    keepMonitoring = True
    t = threading.Thread(target=monitorLocation)
    t.start()

    # Keep iterating over all short stroke distances
    while current < end:

        # Start monitoring the location. If the location exceeds 'current', the robot has overshoot.
        # Define the monitor thread's limits
        monitorX = True
        minX = 0.0
        maxX = current

        pos = ownPosition()
        print "Expected: %8.4f; Actual: %8.4f; Diff: %8.4f" % (0.0, pos[0], abs(pos[0]))
        if violation != None:
            print "Violation: %s" % violation
            violation = None

        # Move to the short stroke position
        move(current, 0, 1.67)

        sleep(0.5)
        pos = ownPosition()
        print "Expected: %8.4f; Actual: %8.4f; Diff: %8.4f" % (current, pos[0], abs(pos[0]-current))
        if violation != None:
            print "Violation: %s" % violation
            violation = None

        # Move back to initial position
        move(0, 0, 1.67)

        current += step

        # report result
        #print "result: vx = %6.2f m/s" % (resultSpeed)

    keepMonitoring = False

