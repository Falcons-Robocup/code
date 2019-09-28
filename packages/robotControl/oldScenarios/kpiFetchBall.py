""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *



def kpiFetchBall(startX=-2, startY=-6, withDispatch=True):
    """
    Sprint towards the ball, fetch it and shoot it back.
    Do this in a loop, from a fixed starting point (default next to penalty spot).
    KPI definition is the time it takes between starting to chase and shooting back the ball.
    So, main contributors are:
      * drive speed
      * ballTracking accuracy while driving
      * ballHandling (rotation / Tokyo drift)
    Optional: put a supporting robot in dispatchBall, to get some reproducability.
    The fetching robot will then wait until the supporting robot shoots.
    """
    # sanitize inputs
    startX = float(startX)
    startY = float(startY)
    phi = 0.5*pi
    withDispatch = int(withDispatch)
    # home: where to pass the ball back to
    # a supporting robot might be there intercepting it
    (homeX, homeY) = (0, -6)
    # iterate
    while True:
        # move to starting location
        move(startX, startY, phi)
        if withDispatch:
            # wait until we see the ball
            if not teamSeesBall():
                continue
            # wait until the ball starts moving straight forward in Y
            s = ballSpeed()
            while (s[1] < 0.2) or (abs(s[0]) > 0.2):
                sleep(0.05)
                s = ballSpeed()
        tStart = time()
        log("starting to fetch", 0)
        # chase the ball
        while True:
            # get the ball if we see it
            if teamSeesBall():
                if hasBall():
                    passToTarget(homeX, homeY)
                    # iteration finished
                    break
                else:
                    getBall()
            else:
                sleep(0.1)
        # iteration complete, report time
        elapsed = time() - tStart
        log("ball fetch iteration result: %6.2f seconds" % (elapsed), 0)
    
    

