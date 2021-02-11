# Copyright 2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import os, sys
import falconspy
import rtdb2tools
import rdlLib

import math
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from matplotlib.patches import Ellipse
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle


class BallPlotter():
    def __init__(self, rdlfiles, agent):
        self.rdlfiles = rdlfiles
        self.agent = agent

    def plot_field(self, plot):
        plot.add_patch(Rectangle((-9.0,-6.0), 18.0, 12.0, fill=False))
        plot.add_patch(Rectangle((-9.0,-3.25), 2.25, 6.5, fill=False))
        plot.add_patch(Rectangle((9.0-2.25,-3.25), 2.25, 6.5, fill=False))
        plot.add_patch(Circle((0.0,0.0), 2.0, fill=False))


    def drawBall(self, plot, rdlfile, agent, color):
        rdl = rdlLib.RDLFile(rdlfile)
        rdl.openRDL()
        lastBall = None

        while True:
            frame = rdl.readNextFrame()
            if frame is None:
                break
        
            if agent in frame.data:
                agentData = frame.data[agent]
                if 'BALLS' in agentData:
                    balls = agentData['BALLS']
                    if(len(balls.value) > 0):
                        bestBall = balls.value[0]                    
                        if(lastBall != None):
                            ballX = -bestBall[0][1]
                            ballY = bestBall[0][0]                                
                            
                            lastX = -lastBall[0][1]
                            lastY = lastBall[0][0]

                            l1 = lines.Line2D([lastX, ballX], [lastY, ballY], color=color, alpha=0.5)
                            plot.add_line(l1)
                            
                        lastBall = bestBall  

    def drawTrueBall(self, plot, rdlfile, agent):
        rdl = rdlLib.RDLFile(rdlfile)
        rdl.openRDL()
        lastBall = None

        while True:
            frame = rdl.readNextFrame()
            if frame is None:
                break
        
            if agent in frame.data:
                agentData = frame.data[agent]
                if 'DIAG_TRUE_BALL' in agentData:
                    tball = agentData['DIAG_TRUE_BALL']
                    if(lastBall != None):
                        ballX = -tball.value['y']
                        ballY = tball.value['x']                            
                        
                        lastX = -lastBall.value['y']
                        lastY = lastBall.value['x']

                        l1 = lines.Line2D([lastX, ballX], [lastY, ballY], color='cyan', alpha=0.5)
                        plot.add_line(l1)
                        
                    lastBall = tball                           



    def run(self):
        figure = plt.figure()
        figure.clf()
        plot = figure.add_subplot(111, aspect='equal')

        self.plot_field(plot)

        colors = ['blue', 'orange', 'green', 'red', 'purple']
        for rdlfile, color in zip(self.rdlfiles, colors):   
            self.drawBall(plot, rdlfile, self.agent, color)
            self.drawTrueBall(plot, rdlfile, self.agent)

        plot.set_xlim(-10, 10)
        plot.set_ylim(-7, 7)
        plot.grid()   
        figure.canvas.draw()

        plt.show()


class ErrorPlotter():
    def __init__(self, rdlfiles, agent):
        self.rdlfiles = rdlfiles
        self.agent = agent

    def drawError(self, plot, rdlfile, agent, color='blue'):
        rdl = rdlLib.RDLFile(rdlfile)
        rdl.openRDL()

        errors = []
        ages = []
        
        while True:
            frame = rdl.readNextFrame()
            if frame is None:
                break
        
            if agent in frame.data:
                agentData = frame.data[agent]

                if 'BALLHANDLERS_BALL_POSSESSION' in agentData and agentData['BALLHANDLERS_BALL_POSSESSION'].value:
                    continue

                if 'DIAG_TRUE_BALL' in agentData and 'BALLS' in agentData:
                    balls = agentData['BALLS']
                    if(len(balls.value) > 0):
                        bestBall = balls.value[0]    
                        tball = agentData['DIAG_TRUE_BALL']
                    
                        ex = bestBall[0][1] - tball.value['y']
                        ey = bestBall[0][0] - tball.value['x']
                        err = math.sqrt(ex*ex + ey*ey)
                        if not math.isnan(err):
                            err = min(1.0, err)
                            errors.append(err)
                            ages.append(frame.age)
                     
        plot.scatter(ages, errors, color=color, edgecolor='black')        


    def run(self):     
        colors = ['blue', 'orange', 'green', 'red', 'purple']

        for rdlfile, color in zip(self.rdlfiles, colors):   
            self.drawError(plt, rdlfile, self.agent, color)

        plt.grid()           

        plt.show()


usage = 'Usage: true_ball_plotter.py [error|field] [agent] rdl1 rdl2...'

if(len(sys.argv) < 4):
    print(usage)
    sys.exit()

plot_type = sys.argv[1]
agent = int(sys.argv[2])
rdlfiles = sys.argv[3:]

if(plot_type == 'error'):
    error_plotter = ErrorPlotter(rdlfiles, agent)
    error_plotter.run()
elif(plot_type == 'field'):
    ball_plotter = BallPlotter(rdlfiles, agent)
    ball_plotter.run()
else:
    print(usage)