# Copyright 2017-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
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
    
