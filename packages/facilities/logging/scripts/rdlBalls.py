""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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
