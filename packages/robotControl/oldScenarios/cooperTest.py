""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



DIRECTION_X = 0 # short side: from -x to +x
DIRECTION_Y = 1 # long side: from -y to +y


def cooperTest(radius, position=2, direction=DIRECTION_X, timeout=240):
    """
    Race from line to line accross the field.
    Part of TechChallenge: run1. 
    For safety reasons, radius is a REQUIRED argument. If we would show this demo on a small field, while the scenario guesses it based on standard MSL field dimensions, we could run full speed into the border.
    """
    # sanitize inputs
    radius = float(radius)
    position = float(position)
    direction = int(direction)
    timeout = float(timeout)
    # choose targets (only apply RCS Y speed, i.e. drive forwards / backwards without rotating, to minimize deviations from intended line)
    if direction == DIRECTION_X:
        target1 = (-radius, position, 0)
        target2 = ( radius, position, 0)
    else:
        target1 = (position, -radius, 0.5*pi)
        target2 = (position,  radius, 0.5*pi)
    # select FAST motion profile
    #setConfig('motionProfile', 'fast') # TODO Erik?
    # execute
    tStart = time()
    elapsed = 0
    targets = [target1, target2]
    count = 0
    while elapsed < timeout:
        move(*targets[count % 2])
        count += 1
        elapsed = time() - tStart
    # show result
    log("number of iterations: %d" % count, 0)

