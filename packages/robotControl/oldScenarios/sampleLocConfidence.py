""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from monitorLocalization import monitorLocalization
import subprocess
import numpy

def getTimeNow():
    EPOCH = 1388534400 # TODO put somewhere common
    return time() - EPOCH

def determineTraceFile():
    logdir = subprocess.check_output("newest_logdir.py", shell=True).strip()
    traceFile = subprocess.check_output("ls -altr1 " + logdir + "/ptrace*wm*| tail -1 | awk '{print $NF}'", shell=True).strip()
    return traceFile

def inspectTraceFile(traceFile):
    tCurr = getTimeNow()
    N = 100
    tCheck = tCurr - 1.0 # take the last few samples
    lines = subprocess.check_output("grep VISION %s | tail -%d" % (traceFile, N), shell=True).split('\n')
    # fill the lists with measurements
    x = []
    y = []
    phi = []
    conf = []
    for line in lines:
        words = line.split()
        if len(words) == 8:
            # NOTE: this data may be flipped w.r.t. worldModel localization
            # but once locked, it should not easily flip
            if words[1] == "VISION":
                t = float(words[0])
                if (t > tCheck):
                    x.append(float(words[3]))
                    y.append(float(words[4]))
                    phi.append(float(words[5]))
                    conf.append(float(words[6]))
    # TODO fix phi / better to avoid (not use phi=0)
    # statistics
    N = len(x)
    if N > 5:
        log("%7d %8.4f(%4.2f) %8.4f(%4.2f) %8.4f(%4.2f) %5.3f" % (N, numpy.mean(x), 3*numpy.std(x), numpy.mean(y), 3*numpy.std(y), numpy.mean(phi), 3*numpy.std(phi), numpy.mean(conf)), 0)
        return [True, numpy.mean(conf)]
    return [False, 0]

def sampleLocConfidence(dx=1, dy=1, xl=-7, xr=7, yl=-10, yr=10):
    """
    Measure vision confidence at a pre-defined grid. 
    """
    # sanitize inputs
    dx = float(dx)
    dy = float(dy)
    PHI_LIST = [0.5*pi, 0.75*pi]
    xl = float(xl)
    xr = float(xr)
    yl = float(yl)
    yr = float(yr)
    # initialize
    global worstConf, worstPos
    worstPos = None
    worstConf = 1.0
    traceFile = determineTraceFile()
    # helper function
    def doit(pos):
        move(*pos)
        sleep(1)
        global worstConf, worstPos
        [success, conf] = inspectTraceFile(traceFile)
        #log('confidence at (%4.1f, %4.1f) = %6.2f' % (pos[0], pos[1], conf), 0)
        if success:
            if conf < worstConf:
                worstConf = conf
                worstPos = pos
        return success
    # sample the grid
    nx = 1 + int((xr - xl) / dx)
    ny = 1 + int((yr - yl) / dy)
    leftToRight = True
    log("%7s %8s(%4s) %8s(%4s) %8s(%4s) %5s" % ('n', '<x>', '3s', '<y>', '3s', '<phi>', '3s', 'conf'), 0)
    for iy in range(ny):
        y = yl + iy * dy
        for ix in range(nx):
            x = xl + ix * dx
            if not leftToRight:
                x = xr - ix * dx
            for phi in PHI_LIST:
                pos = (x, y, phi)
                success = False
                while success == False:
                    success = doit(pos)
        leftToRight = not leftToRight
    # report result
    log('worst confidence: %6.2f at position (%4.1f, %4.1f)' % (worstConf, worstPos[0], worstPos[1]), 0)


