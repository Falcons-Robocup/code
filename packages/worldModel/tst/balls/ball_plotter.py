# Copyright 2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0

import os, sys

import falconspy
import rtdb2tools
from rdlLib import RDLFile

import math
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from matplotlib.patches import Ellipse
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle



class RdlParser():
    def __init__(self, rdlfile):
        self.current_frame = 0
        self.rdl = RDLFile(rdlfile)
        self.rdl.parseRDL()


    def readFrame(self):
        if (self.current_frame + 1) < len(self.rdl.frames):
            self.current_frame = self.current_frame + 1
            return self.rdl.frames[self.current_frame]
        else:
            return None

    def rewind(self):
        self.current_frame = 0
        
class BallPrinter():
    def __init__(self, rdlfile):
        self.key = 'BALL_CANDIDATES_FCS'
        
        self.rdlParser = RdlParser(rdlfile)

    def run(self):
        while True:
            frame = self.rdlParser.readFrame()
        
            for agent in range(0,7):
                if frame.data.has_key(agent):
                    agentData = frame.data[agent]
                    if agentData.has_key(self.key):
                            item = agentData[self.key]
                            print(item)
                            print(dir(item))


class BallPlotter():
    def __init__(self, rdlfile):
        self.rdlParser = RdlParser(rdlfile)

    def plot_field(self, plot):
        plot.add_patch(Rectangle((-6.0,-9.0), 12.0, 18.0, fill=False))
        plot.add_patch(Rectangle((-3.25,-9.0), 6.5, 2.25, fill=False))
        plot.add_patch(Rectangle((-3.25,9.0-2.25), 6.5, 2.25, fill=False))
        plot.add_patch(Circle((0.0,0.0), 2.0, fill=False))


    def sphericalToField(self, sph):
        az = sph[4] + math.pi/2.0
        el = sph[5]
        ra = sph[6]
        cx = sph[7]
        cy = sph[8]
        cz = sph[9]
        cp = sph[10] - math.pi/2.0

        cart = np.zeros((4,4))
        cart[0][3] = ra*math.cos(el)*math.cos(az)
        cart[1][3] = ra*math.cos(el)*math.sin(az)
        cart[2][3] = ra*math.sin(el)
        cart[3][3] = 1.0

        translate = np.eye(4)
        translate[0][3] = cx
        translate[1][3] = cy
        translate[2][3] = 0

        rotatez = np.eye(4)
        rotatez[0][0] = math.cos(cp)
        rotatez[0][1] = -math.sin(cp)
        rotatez[1][0] = math.sin(cp)
        rotatez[1][1] = math.cos(cp)

        M = np.matmul(translate, rotatez)

        resultM = np.matmul(M, cart)

        result = (resultM[0][3], resultM[1][3], resultM[2][3] + cz)
        return result


    def drawMeasurements(self, plot):
        self.rdlParser.rewind()
        while True:
            frame = self.rdlParser.readFrame()
            
            if(frame is None):
                break

            if(frame.age > 40.0 and frame.age < 45.0):
                colors = ['red', 'blue', 'green', 'yellow', 'gray', 'orange', 'brown']
                for agent in range(0,7):
                    if frame.data.has_key(agent):
                        agentData = frame.data[agent]
                        if agentData.has_key('BALL_CANDIDATES_FCS'):
                            items = agentData['BALL_CANDIDATES_FCS']
                            for item in items.value:
                                cameraX = item[7]
                                cameraY = item[8]

                                ball = self.sphericalToField(item)
                                plot.scatter(ball[0], ball[1], c=colors[agent])
                                plot.scatter(cameraX, cameraY, c=colors[agent], marker='^')

                                #l1 = lines.Line2D([cameraX, ball[0]], [cameraY, ball[1]], color=colors[agent], alpha=0.2)
                                #plot.add_line(l1)                              


    def drawBall(self, plot, agent):
        self.rdlParser.rewind()
        lastBall = None

        while True:
            frame = self.rdlParser.readFrame()
            
            if(frame is None):
                break

            if(frame.age > 40.0 and frame.age < 45.0):
                if frame.data.has_key(agent):
                    agentData = frame.data[agent]
                    if agentData.has_key('BALLS'):
                        balls = agentData['BALLS']
                        if(len(balls.value) > 0):
                            bestBall = balls.value[0]                    
                            if(lastBall != None):
                                ballX = bestBall[0][0]
                                ballY = bestBall[0][1]
                                velX = bestBall[1][0] / 3.0
                                velY = bestBall[1][1] / 3.0
                                
                                lastX = lastBall[0][0]
                                lastY = lastBall[0][1]

                                l1 = lines.Line2D([lastX, ballX], [lastY, ballY], color='red', alpha=0.5)
                                plot.add_line(l1)

                                l2 = lines.Line2D([ballX, ballX+velX], [ballY, ballY+velY], color='blue', alpha=0.5)
                                plot.add_line(l2)

                            lastBall = bestBall                           



    def run(self):
        figure = plt.figure()
        figure.clf()
        plot = figure.add_subplot(111, aspect='equal')

        self.plot_field(plot)
        self.drawMeasurements(plot)
        self.drawBall(plot, agent=5)

        plot.set_xlim(-7, 7)
        plot.set_ylim(-10, 10)
        plot.grid()
        figure.canvas.draw()

        plt.show()



#rdlfile = '/media/robocup/5C415A5F563EC7C9/rdls/2019_09_25/own_r5.rdl'
rdlfile = '/media/robocup/5C415A5F563EC7C9/rdls/2019_09_25/own_r5_stim.rdl'

ballPrinter = BallPrinter(rdlfile)
#ballPrinter.run()

ballPlotter = BallPlotter(rdlfile)
ballPlotter.run()