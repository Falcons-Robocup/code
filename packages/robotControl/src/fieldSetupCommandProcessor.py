""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# fieldSetupCommandProcessor
# purpose: listen to g_commandChannelIn and process filtered commands for fieldSetup as sent via command channel 
#
# 
# History:
# v 1.00 MKOE 20160410 - fork from interactiveFieldsetup.py
# v 1.1  MKOE 20160701 - repair for teamplay V2  - todo , still remove old v1 stuff like t_action
# v 1.2  MKOE 20160710 - cleanup for tpv2

#
# usage:  rosrun teamplay fieldSetupCommandProcessor   
# 
#

import sys, os
import traceback
import math
import rospy
import numpy
import random
import subprocess
import roslib
roslib.load_manifest('robotControl')
import FalconsEnv
import FalconsTrace
import FalconsMath
import FalconsCoordinates


from rosMsgs.msg import t_commandChannel
import pickle  #serializer

from worldModel.srv import set_own_ball_location, set_own_ball_locationRequest
from worldModel.srv import set_own_obstacle_location, set_own_obstacle_locationRequest
from worldModel.srv import get_own_location

from teamplay.srv import s_tp_input_command, s_tp_input_commandRequest

from rosMsgs.msg import t_worldmodel_calculated


#Check if simulator running
z=subprocess.call('/opt/ros/jade/bin/rostopic type /simulator/ball >/dev/null 2>/dev/null',shell=True)
if z==0:  #command succeeded
   SimulationMode=True
   print "SIMULATION MODE ON"
else:     #command failed
   SimulationMode=False
   print "SIMULATION MODE OFF"


class Robot:

   def __init__(self):
      self.robotIDstr=ROS_NAMESPACE[-1] # get last char , expected /teamA/robotN  N should be robotNR
      try:
         self.robotID=int( self.robotIDstr )  # convert to int if no number this will fail
      except:
         print "NO ROBOTNR can be derived from ros namespace: %s" % ROS_NAMESPACE
         self.robotID=9999

      rospy.wait_for_service('s_get_own_location')
      self.s_get_own_location=rospy.ServiceProxy('s_get_own_location', get_own_location, persistent=True)

      fullServiceName=ROS_NAMESPACE + "/s_tp_input_command" 
      self.tpControl_srv=rospy.ServiceProxy( fullServiceName, s_tp_input_command, persistent=True)

      self.x = 9999
      self.y = 9999
      self.theta= 9999
      self.getLocation()

   def getID(self):
      return self.robotID

   def getIDstr(self):
      return self.robotIDstr

   def requestCommand( self, commandString ):
      print "Request command: " + commandString 
      reqMsg=s_tp_input_commandRequest()
      reqMsg.command=commandString
      response=self.tpControl_srv( reqMsg )
      return response

   def feedObject( self, theObject ):   #theObject can by either ball or obstacle
      (obsX,obsY) = theObject.getLocation()
      #print "==="
      #print obsX, obsY, self.x, self.y
      hypdeg=FalconsMath.distanceCalc(  obsX, obsY, self.x, self.y )
      theta=math.radians( hypdeg[1] ) - self.theta
      #print "calling publish own with T %0.2f ,  R %0.2f" % ( theta , hypdeg[0] )
      #print "==="
      theObject.publishOwn( theta, hypdeg[0])

   def getLocation(self):
      #                                  service name                service type
      myLoc = self.s_get_own_location()
      if math.isnan(myLoc.ownRobot.Pos.x) or math.isnan(myLoc.ownRobot.Pos.x):
         myLoc.ownRobot.Pos.x = 9999
         myLoc.ownRobot.Pos.y = 9999

      self.x = myLoc.ownRobot.Pos.x
      self.y = myLoc.ownRobot.Pos.y
      self.theta = myLoc.ownRobot.Pos.theta

      return (self.x, self.y, self.theta)

   def setOwnLocation(self, x, y):  # used for interactive mode to be able to set own location by mouseclick
      msgtype = set_own_locationRequest()  
      msgtype.confidence = 0.9
      msgtype.ownRobotPos.x = x
      msgtype.ownRobotPos.y = y
      response = svcpSetOwnLocation(msgtype)

class Obstacle:
   def __init__(self,x,y):
      self.x=x
      self.y=y

   def getLocation(self):
      return (self.x, self.y)

   def publishOwn(self, angle, radius):
      msgenvelope=set_own_obstacle_locationRequest()
      msgenvelope.angle=angle
      msgenvelope.radius=radius
      msgenvelope.confidence=0.9
      msgenvelope.color=0
      ObstacleList.s_set_own_obstacle_location( msgenvelope )

class ObstacleList:

   def __init__( self ):
      rospy.wait_for_service('s_set_own_ball_location')
      #set class variable for reuse by other class
      ObstacleList.s_set_own_obstacle_location = rospy.ServiceProxy('s_set_own_obstacle_location', set_own_obstacle_location, persistent=True)
      self.obstacleList=[]
      self.autoFeed=False
   
   def set( self, coordList ):
      #transform the coordlist to a list of Obstacles
      self.obstacleList=[]
      for coord in coordList:
         self.obstacleList.append( Obstacle( coord[0], coord[1] ) )

   def get(self):
      return self.obstacleList

   def autoFeedOn( self):
      self.autoFeed=True

   def autoFeedOff( self ):
      self.autoFeed=False

   def autoFeedIsOn( self ):
      return self.autoFeed

class Ball:
   def __init__( self):
      rospy.wait_for_service('s_set_own_ball_location')
      self.s_set_own_ball_location = rospy.ServiceProxy('s_set_own_ball_location', set_own_ball_location, persistent=True)
      self.autoFeed=False
      self.x=0.0
      self.y=0.0

   def publishOwn(self, angle, radius):
      if self.autoFeed==True:
         msgenvelope=set_own_ball_locationRequest()
         msgenvelope.angle=angle
         msgenvelope.radius=radius
         msgenvelope.height=0.0
         msgenvelope.confidence=0.9
         self.s_set_own_ball_location( msgenvelope )

   def setLocation( self,x,y ):  #used for interactive mode to emulate ball position
      self.x=x
      self.y=y

   def getLocation(self):
      return (self.x, self.y)

   def autoFeedOn( self):
      self.autoFeed=True

   def autoFeedOff( self ):
      self.autoFeed=False

   def autoFeedIsOn( self ):
      return self.autoFeed

def cbCommandReceived(data):

       #print "callback with data %s" % data

       if thisRobot.getIDstr() in data.to:   # is the message intended for this robot?
          if data.commandType == t_commandChannel.stringCommand:
             commandSplit = data.commandValue.split(' ')
             if commandSplit[0] == 'ball':
                print "set ball to %s , %s" % ( commandSplit[1], commandSplit[2] )
                theBall.setLocation( float( commandSplit[1] ), float( commandSplit[2] ) )
             elif commandSplit[0] == "autoFeedOwnBall":
                if commandSplit[1] == "On":
                   print "autoFeedOwnBall On" 
                   theBall.autoFeedOn()
                else:
                   print "autoFeedOwnBall Off" 
                   theBall.autoFeedOff()

             elif commandSplit[0] == "autoFeedOwnObstacles":
                if commandSplit[1] == "On":
                   print "autoFeedOwnObstacles On" 
                   theObstacleList.autoFeedOn()
                else:
                   print "autoFeedOwnObstacles Off" 
                   theObstacleList.autoFeedOff()

          elif data.commandType == t_commandChannel.controlInterfaceAction:
             print "controlInterfaceAction received:"
             tpControlInterfaceAction=data.commandValue
             print "action:%s " % tpControlInterfaceAction
             thisRobot.requestCommand( tpControlInterfaceAction)

          elif data.commandType == t_commandChannel.controlInterfaceCommand:
             print "controlInterfaceCommand received:"
             tpControlInterfaceCommand=data.commandValue
             print "cmd:%s " % tpControlInterfaceCommand
             response=thisRobot.requestCommand( tpControlInterfaceCommand)
             #we want to send back the response for commands 
             #TODO implement returnchannel

          elif data.commandType == t_commandChannel.controlInterfaceBehavior:
             print "controlInterfaceBehavior received:"
             tpControlInterfaceBehavior=data.commandValue
             print "beh:%s " % tpControlInterfaceBehavior
             thisRobot.requestCommand( tpControlInterfaceBehavior)

          elif data.commandType == t_commandChannel.controlInterfaceRole:
             print "controlInterfaceRole received:"
             tpControlInterfaceRole=data.commandValue
             print "role:%s " % tpControlInterfaceRole
             thisRobot.requestCommand( tpControlInterfaceRole)

          elif data.commandType == t_commandChannel.obstacleList:
             theObstacleList.set( pickle.loads( data.commandValue) )
          else:
             dummy=0


def cbWorldModelCalculated(data):
     # refresh robot location
     thisRobot.getLocation()
     if theBall.autoFeedIsOn():
        thisRobot.feedObject( theBall )

     if theObstacleList.autoFeedIsOn():
        for obstacle in theObstacleList.get():
           thisRobot.feedObject( obstacle )



if __name__ == '__main__': 
    try:
        ROS_NAMESPACE = os.environ['ROS_NAMESPACE']
        theBall=Ball()
        theObstacleList=ObstacleList()
        #robotReasoning=Reasoning()

        print "Ball initialized"
        thisRobot=Robot()
        print "Robot initialized ID: %d " % thisRobot.getID()
        print "================"

 	FalconsTrace.trace( 'ROS NAMESPACE=%s'  % ROS_NAMESPACE ) 

        rospy.init_node('fieldSetupCommandProcessor')  #Running on ROBOT ONLY

        print "Subscribing to g_commandChannelFromCoach"
        rospy.Subscriber("g_commandChannelFromCoach" , t_commandChannel , cbCommandReceived)

        # Subscribe to worldmodel calculated callback
        print "Subscribe to worldmodel heartbeat"
        rospy.Subscriber( "t_worldmodel_calculated", t_worldmodel_calculated, cbWorldModelCalculated)
 
        # spin() simply keeps python from exiting, but will process callbacks until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
       print "ROS exception"
       print traceback.format_exc()
       print sys.exc_info()[0]   
       rospy.signal_shutdown("exception encountered") 
	    
    except:
       print "Error in interactiveFieldSetup.py"
       print traceback.format_exc()
       print sys.exc_info()[0]
       rospy.signal_shutdown("error encountered")


