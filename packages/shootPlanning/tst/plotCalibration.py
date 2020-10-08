""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3
#
# Plot shootPlanning shot calibration values.
# Require file name as argument, e.g. config/shootPlanningShotCalibration.txt
# 
# FALCONS // Jan Feitsma, December 2017


import sys, os
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def loadConfig(filename):
    samples = []
    for line in file(filename).readlines():
        if line[0] != '#':
            words = line.split()
            if len(words) == 4:
                samples.append(tuple([float(w) for w in words]))
    return samples
    

def plot(samples, idx, title):
    fig = plt.figure()
    ax = Axes3D(fig)
    x = [s[0] for s in samples]
    y = [s[1] for s in samples]
    z = [s[idx] for s in samples]
    # plot_wireframe does not look very nice, so just scatter
    ax.scatter(x, y, z)
    ax.set_xlabel('kicker angle')
    ax.set_ylabel('kicker power')
    ax.set_zlabel(title + ' [m]')
    fig.canvas.set_window_title(title)
    fig.canvas.draw()


    
    
# command line interface
if __name__ == '__main__':

    # no argument parsing
    if len(sys.argv) <= 1:
        print("ERROR: missing argument: calibration config file")
        sys.exit(1)
    
    # load data
    samples = loadConfig(sys.argv[1])
    
    # plot
    plot(samples, 2, 'shot height')
    plot(samples, 3, 'shot distance')
    plt.show()
    
