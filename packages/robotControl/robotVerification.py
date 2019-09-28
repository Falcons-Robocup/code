""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python library for verification scenarios.
#
# Jan Feitsma, March 2018



# imports
import numpy
from robotInterface import *



def prompt(msg):
    log(msg, 0)
    log("[hit enter to continue]", 0)
    raw_input()

class ballMonitor():
    """
    Monitor class for worldModel ball position and speed.
    Can calculate the average position and the standard deviation.
    """
    
    def __init__(self):
        self.reset()
        self.active = False
        startThread(self.worker)
        
    def reset(self):
        self.x = []
        self.y = []
        self.z = []
        self.vx = []
        self.vy = []
        self.vz = []
        self.d2 = []
        self.valid = []
        
    def worker(self):
        dt = 0.05 # 20Hz, which is slower than heartbeat (30Hz), to make sure each sample is unique
        while not isShutDown():
            sleep(dt)
            if self.active:
                if teamSeesBall():
                    bpos = ballPosition()
                    vel = ballSpeed()
                    rpos = ownPosition()
                    self.vx.append(vel[0])
                    self.vy.append(vel[1])
                    self.vz.append(vel[2])
                    self.x.append(bpos[0])
                    self.y.append(bpos[1])
                    self.z.append(bpos[2])
                    self.d2.append((rpos[0] - bpos[0])**2 + (rpos[1] - bpos[1])**2)
                    self.valid.append(True)
                else:
                    self.valid.append(False)

    def start(self):
        self.active = True
        self.tStart = time()
        self.reset()
        
    def stop(self):
        self.active = False
        self.tStop = time()
        
    def report(self):
        elapsed = self.tStop - self.tStart
        n = len(self.x)
        log("ball tracking results after %6.2f seconds (%d samples):" % (elapsed, n), 0)
        log("average position  : (%8.4f, %8.4f, %8.4f)" % (numpy.mean(self.x), numpy.mean(self.y), numpy.mean(self.z)), 0)
        log("  position repro  : (%8.4f, %8.4f, %8.4f)" % (3.0 * numpy.std(self.x), 3.0 * numpy.std(self.y), 3.0 * numpy.std(self.z)), 0)
        log("average speed     : (%8.4f, %8.4f, %8.4f)" % (numpy.mean(self.vx), numpy.mean(self.vy), numpy.mean(self.vz)), 0)
        log("  speed repro     : (%8.4f, %8.4f, %8.4f)" % (3.0 * numpy.std(self.vx), 3.0 * numpy.std(self.vy), 3.0 * numpy.std(self.vz)), 0)

# end of class 
# instatiate one object
_monitor = ballMonitor()


def startBallMonitor():
    _monitor.start()
    
def stopBallMonitor():
    _monitor.stop()

def verifyNoBall():
    ok = all([not v for v in _monitor.valid])
    n = len(_monitor.valid)
    if ok:
        log("PASS: no ball detected (%d samples):" % (n), 0)
    else:
        bad = sum(_monitor.valid)
        log("FAIL: %d out of %d samples should not have seen a ball, first position at (%6.2f, %6.2f, %6.2f)" % (bad, n, _monitor.x[0], _monitor.y[0], _monitor.z[0]), 0)

def verifyContinuousBall():
    ok = all(_monitor.valid)
    n = len(_monitor.valid)
    if ok:
        log("PASS: continuous ball detected (%d samples):" % (n), 0)
    else:
        bad = sum([not v for v in _monitor.valid])
        log("FAIL: %d out of %d samples did not see a ball" % (bad, n), 0)

def verifyBallNoiseSmallerThan(noiseLevel):
    noise = max(3.0 * numpy.std(_monitor.x), 3.0 * numpy.std(_monitor.y), 3.0 * numpy.std(_monitor.z))
    if noise < noiseLevel:
        log("PASS: ball repro (%6.2f) smaller than threshold (%6.2f):" % (noise, noiseLevel), 0)
    else:
        log("FAIL: ball repro (%6.2f) exceeds threshold (%6.2f):" % (noise, noiseLevel), 0)
    
def verifyBallCloserThan(distance):
    dist = sqrt(numpy.mean(_monitor.d2))
    if dist < distance:
        log("PASS: ball distance (%6.2f) smaller than threshold (%6.2f):" % (dist, distance), 0)
    else:
        log("FAIL: ball distance (%6.2f) exceeds threshold (%6.2f):" % (dist, distance), 0)
    
def verifyCameraBackTilt():
    z = numpy.mean(_monitor.z)
    dist = sqrt(numpy.mean(_monitor.d2))
    log("ball z = %6.2f" % (z), 0)

