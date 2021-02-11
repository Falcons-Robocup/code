# Copyright 2019 abenabda (Falcons)
# SPDX-License-Identifier: Apache-2.0
import os
import argparse
from rdlLib import RDLFile
import matplotlib.pyplot as plt


# An experiment is done with robot 3 and an rdl file is generated 
# The file (20190829_193432_r3.rdl) is on clipper (not commited)
# The scenario of the expirement is as follow:
#   frames[1567100104, 1567100116] : Collect the ball
#   frames[1567100116, 1567100138] : Move to the center
#   frames[1567100138, 1567100180] : ass the ball to the goal
#   frames[1567100180, 1567100190] : Move to a position where the ball is visible
#   frames[1567100190, 1567100196] : Get the ball
#   frames[1567100196, 1567100217] : Move to the center
#   frames[1567100217, 1567100241] : Shoot the Ball \n The ball hits the robot

key_frames = [1567100072, 1567100104, 1567100116, 1567100138, 1567100180, 1567100190, 1567100196, 1567100217, 1567100241]
timestamps = [key_frames[i] - key_frames[0] for i in range(1, len(key_frames))]

title =["1. Collect the ball", 
        "2. Move to the center", 
        "3. Pass the ball to the goal", 
        "4. Move to a position where the ball is visible", 
        "5. Get the ball", 
        "6. Move to the center", 
        "7. Shoot the Ball \n The ball hits the robot"]

file = "/home/robocup/ball_tracking/exp/190829/20190829_193432_r3.rdl"

k = 0

if __name__ == "__main__":
    # check if file exists
    if not os.path.exists(file):
        print "Error: '%s' not found." % (file)
        exit()
    
    # go through the frames
    rdlFile = RDLFile(file)
    rdlFile.parseRDL()
    frameCounter = 0
    
    ## Collected data
    # timesamp
    T = []
    
    # ball position
    X = []
    Y = []
    Z = []
    
    # ball candidate
    X0 = []
    Y0 = []
    Z0 = []
    
    # robot position   
    RX = []
    RY = []
    RZ = []
    
    for frame in rdlFile.frames:
        for agent in frame.data.keys():
            if len(frame.data[agent]["BALLS"].value):
                if frame.age > timestamps[k+1]:
                    plt.figure
                    plt.subplot(3,1,1) 
                    plt.plot(T, X, 'r.', T, RX, 'k.')
                    plt.title(title[k])
                    plt.subplot(3,1,2) 
                    plt.plot(T, Y, 'g.', T, RY, 'k.')
                    plt.subplot(3,1,3) 
                    plt.plot(T, Z, 'b.', T, RZ, 'k.')
                    plt.show()
                    k += 1
                    T = []
                    X = []
                    Y = []
                    Z = []

                    X0 = []
                    Y0 = []
                    Z0 = []
                    
                    RX = []
                    RY = []
                    RZ = []
                    
                T.append(frame.age)
                # ball position
                X.append(frame.data[agent]["BALLS"].value[0][0][0])
                Y.append(frame.data[agent]["BALLS"].value[0][0][1])
                Z.append(frame.data[agent]["BALLS"].value[0][0][2])
                
                # vision data
                # TODO convert vision data to X and Y
                # X0.append(frame.data[agent]["BALL_CANDIDATES"]...)
                # Y0.append(frame.data[agent]["BALL_CANDIDATES"]...)
                # Z0.append(frame.data[agent]["BALL_CANDIDATES"]...)
                
                # robot position
                RX.append(frame.data[agent]["ROBOT_STATE"].value[2][0])
                RY.append(frame.data[agent]["ROBOT_STATE"].value[2][1])
                RZ.append(frame.data[agent]["ROBOT_STATE"].value[2][2])
                    
        # next
        frameCounter += 1
    
    # plot the last segment
    plt.figure
    plt.subplot(3,1,1) 
    plt.plot(T, X, 'r.', T, RX, 'k.')
    plt.title(title[k])
    plt.subplot(3,1,2) 
    plt.plot(T, Y, 'g.', T, RY, 'k.')
    plt.subplot(3,1,3) 
    plt.plot(T, Z, 'b.', T, RZ, 'k.')
    plt.show()
