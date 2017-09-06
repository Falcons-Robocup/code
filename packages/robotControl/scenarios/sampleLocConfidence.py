""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from monitorLocalization import monitorLocalization
import subprocess


def determineTraceFile():
    logdir = subprocess.check_output("newest_logdir.py", shell=True).strip()
    traceFile = subprocess.check_output("ls -altr1 " + logdir + "/trace*wm* | grep -v Sync | tail -1 | awk '{print $NF}'", shell=True).strip()
    return traceFile

def inspectTraceFile(traceFile):
    N = 10
    values = subprocess.check_output("grep traceVisionMeas %s | tail -%d | awk '{print $(NF-1)}'" % (traceFile, N), shell=True)
    return sum([float(w) for w in values.split()]) / N

def sampleLocConfidence(dx=3, dy=4, phi=0, xl=-6, xr=6, yl=-8, yr=8):
    """
    Measure vision confidence at a pre-defined grid. 
    """
    # sanitize inputs
    dx = float(dx)
    dy = float(dy)
    phi = float(phi)
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
        conf = inspectTraceFile(traceFile)
        log('confidence at (%4.1f, %4.1f) = %6.2f' % (pos[0], pos[1], conf), 0)
        if conf < worstConf:
            worstConf = conf
            worstPos = pos
    # sample the grid
    nx = 1 + int((xr - xl) / dx)
    ny = 1 + int((yr - yl) / dy)
    leftToRight = True
    for iy in range(ny):
        y = yl + iy * dy
        for ix in range(nx):
            x = xl + ix * dx
            if not leftToRight:
                x = xr - ix * dx
            pos = (x, y, phi)
            doit(pos)
        leftToRight = not leftToRight
    # report result
    log('worst confidence: %6.2f at position (%4.1f, %4.1f)' % (worstConf, worstPos[0], worstPos[1]), 0)


