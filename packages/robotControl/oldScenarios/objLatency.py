""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotInterface import *
from robotActions import *



# settings
MEASURE_POSITION = (0, -6, 0.5*pi)
BALL_POSITION = (0, -2, 0)
OBSTACLE_POSITION = (0, -9, 0)
TWIST_ANGLE = 0.2
TWIST_SPEED = 0.5
TWIST_ONLY = True
ENCODERS_ONLY = True


def objLatency():
    """
    Measure ball- and obstacle latency.
    This requires localization latency to have been calibrated.
    """
    
    # prepare
    if ENCODERS_ONLY:
        setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0}) # we will be driving on encoders
    if not TWIST_ONLY:
        move(*MEASURE_POSITION)
        sleep(1)
    
    # make sure a ball and an obstacle are present at the pre-defined positions
    
    # perform twist move
    for it in range(10):
        if TWIST_ONLY:
            duration = TWIST_ANGLE / TWIST_SPEED 
            speed(0, 0, TWIST_SPEED, duration)
            sleep(0.5)
            speed(0, 0, -TWIST_SPEED, duration)
            sleep(0.5)
        else:
            pos1 = list(MEASURE_POSITION)
            pos2 = list(MEASURE_POSITION)
            pos1[2] = pos1[2] - TWIST_ANGLE
            pos2[2] = pos2[2] + TWIST_ANGLE
            move(*pos1)
            sleep(0.5)
            move(*pos2)
            sleep(0.5)

    # done
    restoreConfig("worldModelNode/localization")
    print ""
    print "to analyze and plot: use tracePlot --mode=latency"
    print ""

