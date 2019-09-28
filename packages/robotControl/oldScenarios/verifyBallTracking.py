""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from robotVerification import *



def verifyBallTracking():
    """
    Interactive single-robot sequence to verify worldModel ballTracking tuning.
    """

    # preparation: put robot somewhere on the field and a ball closeby within range
    # wait until inplay and active
    # then start this sequence
    
    # phase 1: static robot, static ball closeby
    startBallMonitor()
    sleep(10)
    stopBallMonitor()
    verifyContinuousBall()
    verifyBallNoiseSmallerThan(0.2)
    verifyBallCloserThan(3)
   
    # phase 2: moving robot, static ball closeby
    startBallMonitor()
    speed(0, 0, 2, 10)
    stopBallMonitor()
    verifyContinuousBall()
    verifyBallNoiseSmallerThan(0.6) # allow some noise due to motion
    verifyBallCloserThan(3)
    
    # phase 3: static robot, put 1 ball far away
    prompt("put one ball in robot view but far away")
    startBallMonitor()
    sleep(10)
    stopBallMonitor()
    verifyContinuousBall() 
    verifyBallNoiseSmallerThan(1.0) # allow some noise far away
    verifyCameraBackTilt() 
    
    # phase 4: robot waypoints to scan the field, no balls visible
    prompt("remove all balls from the field")
    startBallMonitor()
    sleepTime = 3
    strWayPoints = "2 0 0 2 0 3.14 6 1 3.14 6 1 1.57 6 1 -1.57 -6 1 -1.57 -6 1 1.57"
    vecWayPoints = [float(s) for s in strWayPoints.split()]
    n = len(vecWayPoints) / 3
    for it in range(n):
        x = vecWayPoints[3*it]
        y = vecWayPoints[3*it+1]
        phi = vecWayPoints[3*it+2]
        move(x, y, phi) # blocks
        sleep(sleepTime)
    stopBallMonitor()
    verifyNoBall()

