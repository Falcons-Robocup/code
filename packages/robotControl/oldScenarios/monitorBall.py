""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotInterface import *
import numpy



def monitorBall(timeout=10):
    """
    Monitor function for stationary ball.
    Report the average position and the standard deviation.
    """
    dt = 0.05 # 20Hz, which is slower than heartbeat (30Hz), to make sure each sample is unique
    # fill the lists with measurements
    x = []
    y = []
    z = []
    vx = []
    vy = []
    vz = []
    elapsed = 0
    tStart = time()
    while elapsed < timeout:
        sleep(dt)
        pos = ballPosition()
        vel = ballSpeed()
        vx.append(vel[0])
        vy.append(vel[1])
        vz.append(vel[2])
        x.append(pos[0])
        y.append(pos[1])
        z.append(pos[2])
        elapsed = time() - tStart
    # perform statistics
    log("ball tracking results after %6.2f seconds (%d samples):" % (elapsed, len(x)), 0)
    log("average position  : (%8.4f, %8.4f, %8.4f)" % (numpy.mean(x), numpy.mean(y), numpy.mean(z)), 0)
    log("stddev position   : (%8.4f, %8.4f, %8.4f)" % (numpy.std(x), numpy.std(y), numpy.std(z)), 0)
    log("average speed     : (%8.4f, %8.4f, %8.4f)" % (numpy.mean(vx), numpy.mean(vy), numpy.mean(vz)), 0)
    log("stddev speed      : (%8.4f, %8.4f, %8.4f)" % (numpy.std(vx), numpy.std(vy), numpy.std(vz)), 0)


