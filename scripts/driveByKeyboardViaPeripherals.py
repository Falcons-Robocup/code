# Author: Erik Kouters
# Date: 2015-12-22 / Update 2017-10-24
#
# This script publishes to g_robotspeed and s_kick_speed
# Before using this script, make sure to start:
#   rosrun peripheralsInterface motors
#   rosrun velocityControl velocityControl
#      Disable worldmodel adapter
#   rosrun ballHandling ballHandling
#      Disable worldmodel adapter
#
# Keys:
# up,down,left,right -- drive
# p -- (plus) increase speed by 0.5
# m -- (minus) decrease speed by 0.5
# + -- (plus) increase shoot power by 25.0
# - -- (minus) decrease shoot power by 25.0
# l -- (Lob) set kicker height to lob
# g -- (ground) set kicker height to ground

#!/usr/bin/python

import os
import sys, time
import Tkinter as tk
import threading

import roslib; 
roslib.load_manifest('peripheralsInterface') 
import rospy
from rosMsgs.msg import t_robotspeed

from peripheralsInterface.srv import s_peripheralsInterface_setKickSpeed
from peripheralsInterface.srv import s_peripheralsInterface_setKickPosition

setpointPublishing = True

rospy.init_node("someNode", anonymous=True)
pub = rospy.Publisher('g_robotspeed', t_robotspeed, queue_size=10)
VelStepSize = [1.0]

msg=t_robotspeed()

vel = [0.0, 0.0, 0.0]
vel[0] = 0.0 #x
vel[1] = 0.0 #y
vel[2] = 0.0 #phi
shootPower = 25.0

def spaceKeyPress(event):
   print "Shooting..."
   rospy.wait_for_service('/teamA/robot2/s_kick_speed')
   rospy.wait_for_service('/teamA/robot2/s_kick_speed')
   shootService = rospy.ServiceProxy('/teamA/robot2/s_kick_speed', s_peripheralsInterface_setKickSpeed)
   shootService(shootPower)

def printVel():
   msg.vx = vel[0]
   msg.vy = vel[1]
   msg.vphi = vel[2]
   pub.publish(msg)
   print vel

def leftKeyPress(event):
   vel[2] = VelStepSize[0] * 3.0
   printVel()
      
def leftKeyRelease(event):
   vel[2] = 0.0
   printVel()
    
def rightKeyPress(event):
    vel[2] = -VelStepSize[0] * 3.0
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
   while(True):
      if not setpointPublishing:
         break
      pub.publish(msg)
      time.sleep(0.1)


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

def equalKeyPress(event):
    global shootPower
    shootPower += 25.0
    if shootPower > 100.0:
        shootPower = 100.0
    print "shootPower increased to:", shootPower

def minusKeyPress(event):
    global shootPower
    shootPower -= 25.0
    if shootPower < 25.0:
        shootPower = 25.0
    print "shootPower decreased to:", shootPower

def lKeyPress(event):
   rospy.wait_for_service('/teamA/robot2/s_kick_position')
   shootHeight = rospy.ServiceProxy('/teamA/robot2/s_kick_position', s_peripheralsInterface_setKickPosition)
   shootHeight(75.0)

def gKeyPress(event):
   rospy.wait_for_service('/teamA/robot2/s_kick_position')
   shootHeight = rospy.ServiceProxy('/teamA/robot2/s_kick_position', s_peripheralsInterface_setKickPosition)
   shootHeight(0.0)

def keyup(e):
    print 'up', e.keysym
    print 'up', e.char
def keydown(e):
    print 'down', e.keysym
    print 'down', e.char

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
       main.bind('<KeyPress-space>', spaceKeyPress)
       main.bind('<KeyPress-minus>', minusKeyPress)
       main.bind('<KeyPress-equal>', equalKeyPress)
       main.bind('<KeyPress-l>', lKeyPress)
       main.bind('<KeyPress-g>', gKeyPress)

       #main.bind("<KeyPress>", keydown)
       #main.bind("<KeyRelease>", keyup)
         
       t = threading.Thread(target=publishSetpoint)
       t.start()

       main.mainloop()
   finally:
        print("exiting...")
        setpointPublishing = False
        os.system("xset r on")   
