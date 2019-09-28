""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # Author: Erik Kouters
# Date: 2015-12-22
#
# This script publishes to g_robotspeed.
#
# Keys:
# up,down,left,right -- drive
# p -- (plus) increase speed by 0.5
# m -- (minus) decrease speed by 0.5

import os
import sys, time
import Tkinter as tk
import threading

import roslib; roslib.load_manifest('peripheralsInterface') 
import rospy
import numpy
from rosMsgs.msg import t_robotspeed

setpointPublishing = True

FREQUENCY = 100.0

xy_acceleration = 1.6 / FREQUENCY
phi_acceleration = 4.0 / FREQUENCY

rospy.init_node("someNode", anonymous=True)
pub = rospy.Publisher('/teamA/robot3/g_robotspeed', t_robotspeed, queue_size=10)
VelStepSize = [1.0]

msg=t_robotspeed()

vel = [0.0, 0.0, 0.0]
vel[0] = 0.0 #x
vel[1] = 0.0 #y
vel[2] = 0.0 #phi

def printVel():
   msg.vx = vel[0]
   msg.vy = vel[1]
   msg.vphi = vel[2]
   pub.publish(msg)
   print vel

def leftKeyPress(event):
   vel[2] = VelStepSize[0]
   printVel()
      
def leftKeyRelease(event):
   vel[2] = 0.0
   printVel()
    
def rightKeyPress(event):
    vel[2] = -VelStepSize[0]
    printVel()
    
def rightKeyRelease(event):
    vel[2] = 0.0
    printVel()
    
def upKeyPress(event):
    vel[1] = VelStepSize[0]
    printVel()
    
def upKeyRelease(event):
    vel[1] = 0.0
    printVel()
    
def downKeyPress(event):
    vel[1] = -VelStepSize[0]
    printVel()
    
def downKeyRelease(event):
    vel[1] = 0.0
    printVel()

def pKeyPress(event):
    VelStepSize[0] += 0.5

def mKeyPress(event):
    VelStepSize[0] -= 0.5
def qKeyPress(event):
    print("quit.")
    os.system("xset r on")    
    exit()


    
def publishSetpoint():
   state = t_robotspeed()
   state.vx = 0
   state.vy = 0
   state.vphi = 0

   while(True):
      if not setpointPublishing:
         break

      state.vx += numpy.sign(msg.vx - state.vx) * xy_acceleration
      if abs(state.vx - msg.vx) < xy_acceleration:
         state.vx = msg.vx

      state.vy += numpy.sign(msg.vy - state.vy) * xy_acceleration
      if abs(state.vy - msg.vy) < xy_acceleration:
         state.vy = msg.vy

      state.vphi += numpy.sign(msg.vphi - state.vphi) * phi_acceleration
      if abs(state.vphi - msg.vphi) < phi_acceleration:
         state.vphi = msg.vphi

      print(state)

      pub.publish(state)
      time.sleep(1.0 / FREQUENCY)

def dotKeyPress(event):
    vel[0] = VelStepSize[0]
    printVel()

def dotKeyRelease(event):
    vel[0] = 0.0
    printVel()

def commaKeyPress(event):
    vel[0] = -VelStepSize[0]
    printVel()

def commaKeyRelease(event):
    vel[0] = 0.0
    printVel()

def keyup(e):
    print 'up', e.keysym
def keydown(e):
    print 'down', e.keysym

if __name__ == "__main__":

   try:
       os.system("xset r off")
   
       main = tk.Tk()
       main.bind('<KeyPress-Left>', leftKeyPress)
       main.bind('<KeyRelease-Left>', leftKeyRelease)
       main.bind('<KeyPress-Right>', rightKeyPress)
       main.bind('<KeyRelease-Right>', rightKeyRelease)
       main.bind('<KeyPress-Up>', upKeyPress)
       main.bind('<KeyRelease-Up>', upKeyRelease)
       main.bind('<KeyPress-Down>', downKeyPress)
       main.bind('<KeyRelease-Down>', downKeyRelease)
       main.bind('<KeyRelease-p>', pKeyPress)
       main.bind('<KeyRelease-m>', mKeyPress)
       main.bind('<KeyRelease-q>', qKeyPress)
       main.bind('<KeyPress-comma>', commaKeyPress)
       main.bind('<KeyRelease-comma>', commaKeyRelease)
       main.bind('<KeyPress-period>', dotKeyPress)
       main.bind('<KeyRelease-period>', dotKeyRelease)

       #main.bind("<KeyPress>", keydown)
         
       t = threading.Thread(target=publishSetpoint)
       t.start()

       main.mainloop()
   finally:
        print("exiting...")
        setpointPublishing = False
        os.system("xset r on")   
