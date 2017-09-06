""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from monitorSpeed import monitorSpeed


# TODO: this looks a bit duplicate w.r.t. kpiEncoderY
# MVEH: only difference: drive with vision, not purely on encoders
# I thought another difference is that this accelerate before starting measurement to be at constant velocity, but kpiEncoderY also does that with the monitorSpeed
# Could make the "use vision" another input to have 1 scenario?
def kpiSpeedSetPointY(vy = 1, duration = 5, iterations = 3):
    """
    Check speed setpoint and feedback accuracy.
    Accelerate to setpoint, drive in straight line for given duration, check delta position, calculate actual speed
 
    """
    # Since "speed" is used, need to ensure time * speed is within reasonable limits so we don't hit the wall
    if float(vy) * float(duration) > 15.0:
        raise Exception("expected distance more than 15 meter, exceeds safety limit")
    # ok to continue
    averageVy = 0.0
    for iteration in range(0,int(iterations)):
        log('starting test %6.0f' % iteration, 0)		
        # assume initial position 
        log('moving to initial position.', 0)
        move(0, -8, pi*0.5)
        log('starting acceleration.', 0)
        speed(0, float(vy), 0, 1, False)	
        # acceleration phase should be finished
        log('starting measurement.', 0)
        startPos = getPosition()
        speed(0, float(vy), 0, float(duration), True)	
        log('stopping measurement.', 0)
        endPos = getPosition()
        distanceY = endPos[1] - startPos[1]
        realVy = distanceY / float(duration)
        averageVy += realVy		
        log("finished test iteration: %6.0f; set vy = %6.2f, real vy = %6.2f" % (iteration, float(vy), realVy), 0)
        sleep(0.5) # give the robot a moment to stop, otherwise next move in opposite direction will jerk it
        print "Set vy = %6.2f, real vy = %6.2f" % (float(vy), realVy)
    averageVy = averageVy / int(iterations)
    log("finished test; set vy = %6.2f, average real vy = %6.2f" % (float(vy), averageVy), 0)

def kpiEncoderY(vy = 1):
    """
    Check encoder setpoint and feedback accuracy.
    """
    # sanitize inputs
    vy = float(vy)
    # assume initial position 
    move(0, -8, pi*0.5)
    # disable vision by setting the weight to 0
    setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0})
    # start the monitor thread
    startThread(monitorSpeed, -6, 3)
    # drive forwards
    t = 13.0 / vy
    speed(0, vy, 0, t)
    # get current position according to pure encoder displacement
    encoderPos = getPosition()
    # restore config, give worldModel some time to lock on vision again
    restoreConfig("worldModelNode/localization")
    sleep(5)
    visionPos = getPosition()
    # report position delta
    distance = sqrt((visionPos[0] - encoderPos[0])**2 + (visionPos[1] - encoderPos[1])**2)
    print "encoder-only position: x=%6.2f y=%6.2f phi=%6.2f" % (encoderPos[0], encoderPos[1], encoderPos[2])
    print "vision position      : x=%6.2f y=%6.2f phi=%6.2f" % (visionPos[0], visionPos[1], visionPos[2])
    print "mismatch in distance : d=%6.2f" % (distance)


