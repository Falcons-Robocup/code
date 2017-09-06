""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# interactiveFieldSetup
# purpose: graphical interactive way to set ball, set opponents, move own robots etc
#          basic code from reasoning viewer
#
# 
# History:
# v 1.00 MKOE 20160321 - fork from reasoning.py


#
# usage:  rosrun teamplay interactiveFieldSetup 
# 
#

import sys, os
import traceback
import math
import socket   # to get hostname
import rospy
import numpy
import random
import roslib; roslib.load_manifest('teamplay')
import subprocess
import textwrap
import FalconsEnv
import FalconsTrace
import FalconsMath
import FalconsCoordinates
import EnvironmentField      #defined in pymodule in environment facility
from EnvironmentField import *

#from interactiveFieldSetup import reasoning_definitions
#from reasoning.xml_taskreader import readXMLTaskFile
#from reasoning.taskFunctions import *
from interactiveFieldSetup.genericFunctions import *


# http://wiki.ros.org/rospy/Overview/Services
from rosMsgs.msg import t_action, t_sim_position, t_state, t_role, t_ball_in_play
from worldModel.srv import get_own_location, get_ball_location, get_teammembers, get_own_obstacles
from worldModel.srv import set_remote_obstacle_location, set_remote_obstacle_locationRequest
from worldModel.srv import set_own_obstacle_location, set_own_obstacle_locationRequest
from worldModel.srv import set_own_ball_location, set_own_ball_locationRequest
from rosMsgs.msg import t_worldmodel_calculated, t_worldmodel
#from WorldModel.srv import get_ball_possession    #MKOE 20150329 replaced by s_tp_get_ball_posession
#from worldModel.srv import get_opponents   #old, replaced by internal s_tp_
#from rosMsgs.msg import BallPossession   #import to get the definitions from the .msg file

from rosMsgs.msg import t_commandChannel
import pickle  #serializer


TURTLEROOT=os.environ['TURTLEROOT']
sys.path.append( TURTLEROOT + "/packages/coachCommands" )

import argparse
import time, datetime

#Check if simulator running
print "Simulator check...."
from simulator.srv import TeleportBall,TeleportBallRequest

try:
   rospy.wait_for_service('/simulator/teleportball',5000) #wait max 5 seconds to detect simulator precense 
   simulatorTeleportBall=rospy.ServiceProxy('/simulator/teleportball', TeleportBall, persistent=True)
   SimulatorRunning=True
   print "SIMULATOR RUNNING"
except Exception as ex:
   template = "An exception of type {0} occured. Arguments:\n{1!r}"
   message = template.format(type(ex).__name__, ex.args)
   print message
   SimulatorRunning=False  
   print "NO simulator running, standard operating mode"


import pygame
from interactiveFieldSetup.reasoning_viewer import GraphField
from worldModel.srv import set_remote_ball_location, set_remote_ball_locationRequest, set_own_location, set_own_locationRequest

'''
#read/generate translationtable for actions numbers into strings
msgFilePath= TURTLEROOT + "/packages/facilities/rosMsgs/msg/t_action.msg"
with open( msgFilePath ) as msgFile:
   content = msgFile.readlines()

# lines look like: (without the first #)
#uint16 ADV_MOVE_TO_POS = 10020           #expects 6 or 3params: posX,posY,theta,velX,velY,velZ
actionTable={}

for line in content:
   defLine=line.split()
   if len(defLine)>=4:
      if defLine[0][0] != "#":  #filter out comment lines
         actionName=defLine[1]
         actionNumber=defLine[3]
         actionTable[actionNumber]=actionName

print "Actiontable"
print actionTable
'''

class F_POI:
   def __init__(self):
      self.poiList=StringVector()
      self.myField=EnvironmentField.cEnvironmentField.getInstance()
      self.myField.getFieldPOIList( self.poiList )
      print "Number of POIs: ", self.poiList.size()
      self.poiDrawIndex=0

   def getPOI(self, name):
      myPOI=EnvironmentField.poiInfo() # prepare myPOI structure
      self.myField.getFieldPOIByString( name, myPOI)
      return ( (myPOI.x, myPOI.y) )

   def printPOIList( self ):
      for poiName in self.poiList:
        print "%s : %s" % ( poiName, self.getPOI( poiName ))

   def POIList(self):
      return self.poiList

   def plotPOI(self, graphFieldObject, poiName, windowName):
       graphFieldObject.drawFieldPoint( self.getPOI( poiName ) , windowName )

   def plotPOIs(self, graphFieldObject): #draw all needed POIs for field display
       for poiName in [ "P_OWN_PENALTY_SPOT", "P_OPP_PENALTY_SPOT" ] : 
         self.plotPOI( graphFieldObject, poiName, 'field')

   def drawPOIFeature(self, graphFieldObject):
      myPOI=self.poiList[ self.poiDrawIndex ]
      self.plotPOI( graphFieldObject, myPOI, 'feature')
      graphFieldObject.boxText( "featureName", [ "POI: " + myPOI ] )

   def drawNextPOIFeature(self, graphFieldObject):
      self.poiDrawIndex= self.poiDrawIndex+1
      if self.poiDrawIndex >= self.poiList.size():
         self.poiDrawIndex=0
      self.drawPOIFeature( graphFieldObject )

   def drawPreviousPOIFeature(self, graphFieldObject):
      self.poiDrawIndex= self.poiDrawIndex-1
      if self.poiDrawIndex <0  :
         self.poiDrawIndex=self.poiList.size() -1
      self.drawPOIFeature( graphFieldObject )

class F_Areas:
   def __init__(self):
      self.areaList=StringVector()
      self.myField=EnvironmentField.cEnvironmentField.getInstance()
      self.myField.getFieldAreaList( self.areaList )
      print "Number of Areas: ", self.areaList.size()
      self.areaDrawIndex=0

   def getArea(self, name):
      myArea=EnvironmentField.areaInfo() # prepare myArea structure
      self.myField.getFieldAreaByString( name, myArea)
      return  myArea 

   def printAreaList( self ):
      for areaName in self.areaList:
        print "%s : %s" % ( areaName, getArea )
      
   def AreaList(self):
      return self.areaList

   def drawAreaFeature(self, graphFieldObject):
      myArea=self.areaList[ self.areaDrawIndex ]
      self.drawArea( graphFieldObject, myArea , 'feature' )
      graphFieldObject.boxText( "featureName", [ "AREA: " + myArea ] )

   def drawNextAreaFeature(self, graphFieldObject):
      self.areaDrawIndex= self.areaDrawIndex+1
      if self.areaDrawIndex >= self.areaList.size():
         self.areaDrawIndex=0
      self.drawAreaFeature( graphFieldObject )

   def drawPreviousAreaFeature(self, graphFieldObject):
      self.areaDrawIndex= self.areaDrawIndex-1
      if self.areaDrawIndex <0  :
         self.areaDrawIndex=self.areaList.size() -1
      self.drawAreaFeature( graphFieldObject )


   def drawArea(self, graphFieldObject, areaName, windowName):
      myArea=self.getArea( areaName )
      areaType= myArea.typeC   #type R='rectangle', C='circle', T='triangle', S='semicircle'
      if areaType == 'R':
         graphFieldObject.drawFieldRectangle( (myArea.R.corner1.x, myArea.R.corner1.y), (myArea.R.corner2.x, myArea.R.corner2.y), windowName )
      elif areaType == 'C':
         graphFieldObject.drawFieldCircle( (myArea.C.center.x, myArea.C.center.y), myArea.C.radius , windowName)
      elif areaType == 'T':
         graphFieldObject.drawFieldPoly( ((myArea.T.corner1.x, myArea.T.corner1.y), (myArea.T.corner2.x, myArea.T.corner2.y),(myArea.T.corner3.x, myArea.T.corner3.y)), windowName)


   def drawAreas(self, graphFieldObject):  #draw all needed areas for field display
       #graphFieldObject is of class graphField / defined in reasoningviewer.py
       for area in [ "A_FIELD", "A_FIELD_SAFETY_BOUNDARIES", "A_OWN_SIDE", "A_OPP_SIDE", "A_CENTER_CIRCLE", "A_OWN_GOAL", "A_OPP_GOAL", "A_OPP_PENALTYAREA", "A_OWN_PENALTYAREA", "A_OWN_GOALAREA", "A_OPP_GOALAREA" ] : 
          self.drawArea( graphFieldObject, area, 'field' )


class Robot:
   def __init__( self,robotNumber ):
      self.robotID= robotNumber
      self.x=9999
      self.y=9999
      self.phi=9999
      self.shootPower=100
      self.setpointX=9999
      self.setpointY=9999
      self.setpointFaceX=9999
      self.setpointFaceY=9999
      #self.setpointPhi=9999
      self.isRobotAlive=False
      self.previousSerializedAction="NONE"
      self.ballPossession=False

   def setHaveBall(self, value):
      self.ballPossession=value

   def hasBall(self):
      return self.ballPossession

   def sendTPControlCommandToRobot( self, tpControlCommand ):
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotID
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.controlInterfaceCommand
      commandChannelMsg.commandValue=tpControlCommand
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def sendTPControlActionToRobot( self, tpControlAction ):
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotID
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.controlInterfaceAction
      commandChannelMsg.commandValue=tpControlAction
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def sendTPControlBehaviorToRobot( self, tpControlBehavior ):
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotID
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.controlInterfaceBehavior
      commandChannelMsg.commandValue=tpControlBehavior
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def sendTPControlBehaviorToRobot( self, tpControlBehavior ):
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotID
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.controlInterfaceBehavior
      commandChannelMsg.commandValue=tpControlBehavior
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )


   def sendTPControlCommandToRobot( self, tpControlRole):
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotID
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.controlInterfaceRole
      commandChannelMsg.commandValue=tpControlRole
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def setShootPower(self,power):
      self.shootPower=power

   def getShootPower(self):
      return self.shootPower

   def isAlive(self):
      return self.isRobotAlive
  
   def setAlive(self,value):
      self.isRobotAlive=value

   def feedObstacle( self, obstacle ):
      (obsX,obsY) = obstacle.getLocation()
      #print obsX, obsY, self.x, self.y
      hypdeg=FalconsMath.distanceCalc( self.x, self.y, obsX, obsY )
      theta=math.radians( hypdeg[1] - 180 )  
      #print theta
      obstacle.publishOwn( theta, hypdeg[0])

   def setCurrentLocation( self,x,y,phi ):  #used for interactive mode to emulate ball position
      self.x=x
      self.y=y
      self.phi=phi

   def getCurrentLocation( self ):  #used for interactive mode to emulate ball position
      return ( self.x, self.y, self.phi )

   def setLocation( self,targetX,targetY, faceX, faceY ): 
      self.setpointX=x
      self.setpointY=y
      self.setpointPhi=phi
      self.setpointFaceX=faceX
      self.setpointFaceY=faceY
      commandString="action move target=coord:%0.1f,%0.1f facing=coord:%0.1f,%0.1f" % ( targetX, targetY, faceX, faceY )
      self.sendTPControlActionToRobot( commmandString )

 
class Obstacle:
   def __init__(self,number,x,y):
      self.initialX=x
      self.initialY=y
      self.x=x
      self.y=y
      self.number=number

   def setLocation(self,x,y):
      self.x=x
      self.y=y

   def reset(self):
      self.x=self.initialX
      self.y=self.initialY

   def getLocation(self):
      return (self.x, self.y)

   def getNumber(self):
      return (self.number)

   def publishRemote(self):
      msgenvelope=set_remote_obstacle_locationRequest()
      msgenvelope.obstaclePos.x=float( self.x )
      msgenvelope.obstaclePos.y=float( self.y )
      msgenvelope.obstaclePos.theta=0.0
      msgenvelope.confidence=1.0
      svcpSetRemoteObstacleLocation( msgenvelope )

   def publishOwn(self, angle, radius):
      msgenvelope=set_own_obstacle_locationRequest()
      msgenvelope.angle=angle
      msgenvelope.radius=radius
      msgenvelope.confidence=0.9
      msgenvelope.color=0
      svcpSetOwnObstacleLocation( msgenvelope )

class ObstacleList:
   def __init__(self, robotNR):
      self.robotNR=robotNR
      self.autoFeedOwnObstaclesOff()
      self.obstacleList=[ Obstacle(0,0,9), Obstacle(1,1,3), Obstacle(2,-1,3), Obstacle(3,-2,7), Obstacle(4,2,7)]
      self.getObstacleCoordList()

   def get(self):
      return self.obstacleList

   def toggleFeedOwnObstacles( self ):
      if self.autoFeedOwnObstacles == False:
         self.autoFeedOwnObstacles = True
         self.autoFeedOwnObstaclesOn()
      else:
         self.autoFeedOwnObstacles = False
         self.autoFeedOwnObstaclesOff()

   def autoFeedIsOn( self ):
      return self.autoFeedOwnObstacles

   def feedStatus( self):
      return "autoObstacleFeed: %s" % self.autoFeedOwnObstacles

   def getObstacleCoordList( self ):
      self.obstCoordList=[]
      for theObstacle in self.obstacleList:
         obstCoord=theObstacle.getLocation()
         self.obstCoordList.append( obstCoord )
      return self.obstCoordList

   def sendObstacleListToRobot( self ):
      print "R%d :Sending obstaclelist" % self.robotNR
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to="%s" % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.obstacleList
      commandChannelMsg.commandValue=pickle.dumps( self.getObstacleCoordList() )   #serialize
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )
   
   def autoFeedOwnObstaclesOn( self ):
      self.autoFeedOwnObstacles=True
      print "R%d :autoFeedOwnObstacles On" % self.robotNR
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.stringCommand
      commandChannelMsg.commandValue="autoFeedOwnObstacles On"
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )
      self.sendObstacleListToRobot()

   def autoFeedOwnObstaclesOff( self ):
      self.autoFeedOwnObstacles=False
      print "R%d :autoFeedOwnObstacles Off" % self.robotNR
      commandChannelMsg = t_commandChannel()
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.stringCommand
      commandChannelMsg.commandValue="autoFeedOwnObstacles Off"
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )


class Ball:

   def __init__( self, robotNR ):
      self.robotNR=robotNR
      self.autoFeedOwnBallOff()
      self.x=0
      self.y=0

   def toggleFeedOwnBall( self ):
      if self.autoFeedOwnBall == False:
         self.autoFeedOwnBall = True
         self.autoFeedOwnBallOn()
      else:
         self.autoFeedOwnBall = False
         self.autoFeedOwnBallOff()
 
   def autoFeedIsOn( self ):
      return self.autoFeedOwnBall

   def feedStatus( self):
      return "autoBallFeed: %s" % self.autoFeedOwnBall


   def autoFeedOwnBallOn( self ):
      print "R%d :autoFeedOwnBall On" % self.robotNR
      self.autoFeedOwnBall=True
      commandChannelMsg = t_commandChannel() 
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.stringCommand
      commandChannelMsg.commandValue="autoFeedOwnBall On"
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def autoFeedOwnBallOff( self ):
      print "R%d :autoFeedOwnBall Off" % self.robotNR
      self.autoFeedOwnBall=False
      commandChannelMsg = t_commandChannel() 
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.stringCommand
      commandChannelMsg.commandValue="autoFeedOwnBall Off"
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

   def getLocation( self ):
      return(self.x,self.y)

   def setSimLocation( self,x,y ): 
      if SimulatorRunning:
         print "Teleportball to %s,%s" % ( x, y)
         msg = TeleportBallRequest()  
         msg.x = x
         msg.y = y
         msg.vx = 0
         msg.vy = 0
         response = simulatorTeleportBall(msg)
      else:
         print "Teleportball request without simulator Running skipped!"

   def setLocation( self,x,y ):  #used for interactive mode to emulate ball position
      self.x=x
      self.y=y
      commandChannelMsg = t_commandChannel() 
      commandChannelMsg.sender='C'
      commandChannelMsg.to='%s' % self.robotNR
      commandChannelMsg.commandOwner='fieldSetup'
      commandChannelMsg.commandType=t_commandChannel.stringCommand
      commandChannelMsg.commandValue="ball %s %s" % ( x, y )
      commandChannelMsg.commandID=0 # not used by this program
      commandChannelHandle.publish( commandChannelMsg )

      print "R%d set: %s " % ( self.robotNR, commandChannelMsg.commandValue)

class robotWorld:
   robot=0
   ball=0
   obstacleList=0

class rosLoop:
    
    def __init__(self):

        self.selectedRobot='1'

        self.myAreas= F_Areas()
        self.myPOIs=  F_POI()

        self.robotArray={}
        for robotNR in range(1,7):
           instance=robotWorld()
           instance.robot=Robot(robotNR)
           instance.ball=Ball(robotNR)
           instance.obstacleList=ObstacleList( robotNR )
           self.robotArray[ "%s" % robotNR ]= instance


	teamColor='cyan'
	fieldMirror=False

	self.gameView=GraphField( "Falcons interactive field setup " , teamColor , fieldMirror) 

        pygame.key.set_repeat( 400,150 )   #keyrepeat: wait 400 ms before it genereates keydown events per 100ms

        self.gameView.boxText ( "help", [
                               "1-5 = select robot",
                               "right mouse + r= move robot and turn towards move direction",
                                "SHIFT + o = toggle autoObstacleFeed to  own worldmodel",
                               "CTRL + g = action getBall, CTRL+ALT+g = behavior goalkeeper", 
                               "CTRL + s = action shoot",
                               "                           CTRL+ALT+d = behavior defend",
                               "CTRL + m = action moveToFreeSpot",
                               "CTRL + i = action interceptBall",
                               "CTRL + SHIFT + s = STOP ",
                               "CTRL + SHIFT + r = RESET command interface overrule ",
                               "SHIFT + b = toggle autoballFeed to  own worldmodel",
                               "left mouse + r = move robot without turning",
                               "left mouse + b = set ball position",
                               "left mouse + t = teleport SIMBALL",
                               "left mouse + number key 0-5 = set obstacle location", 
                               "o + r = obstacle reset to initial positions",
                               "i = toggle info window",
                               "f = toggle feature window; arrow L/R=prev/next feature; a=area,p=poi",
                               "(TPv1) KP + = increase shootpower +1  (SHIFT = +10)",
                               "(TPv1) KP - = decrease shootpower -1  (SHIFT = -10)",

                              ] )
	self.pois=F_POI()

	self.pois.plotPOIs( self.gameView )
	self.myAreas.drawAreas( self.gameView )

        self.gameView.drawLine( self.pois.getPOI("P_OPP_CORNER_LEFT"), self.pois.getPOI("P_OWN_CORNER_RIGHT"), (255,0,0), 'cross')
        self.gameView.drawLine( self.pois.getPOI("P_OPP_CORNER_RIGHT"), self.pois.getPOI("P_OWN_CORNER_LEFT"), (255,0,0), 'cross')



        self.LeftMouseDown=False  
        self.RightMouseDown = False
        self.showInfoWindow=False
        self.showHelpWindow=False
        self.showFeatureWindow=False
        self.showPOIFeatures=False
        self.showAreaFeatures=True

        # Subscribe to worldmodel calculated callback 
        print "Subscribe to worldmodel heartbeat"
        rospy.Subscriber( rosNamespace + "t_worldmodel_calculated", t_worldmodel_calculated, self.cbWorldModelCalculated)

        # Subscribe to worldmodel consolidated feed on Coach
        print "Subscribe to worldmodel consolidated data feed"
        rospy.Subscriber( rosNamespace + "g_worldmodel", t_worldmodel, self.cbWorldModelFeed)

            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        print "Ready to rock!"

    def cbWorldModelFeed(self, data):
        robotList=range(1,7)

        myRobots=data.friends
        for bot in myRobots:
           robotInstance=self.robotArray[ "%s" % bot.id ]

           #theRobot=self.robotArray[ "%s" % bot.id ].robot
           theRobot=robotInstance.robot
           theRobot.setCurrentLocation( float(bot.x), float(bot.y), float(bot.phi) )
           if theRobot.isAlive()==False:
              #just before the robot was not alive so now it is alive we need to initialize it with important stuff
              print "R%d : robot alive" % bot.id
              theRobot.setAlive(True)

           robotList.remove( int( bot.id ) )  #remove from robotList as being seen
           if data.ballPossession.robotID==bot.id and data.ballPossession.type == 3:
              theRobot.setHaveBall( True )
           else:
              theRobot.setHaveBall( False )

        #set non reported bots to not Alive
        for missingBotNR in robotList:
           robotInstance=self.robotArray[ "%s" % missingBotNR ]
           theRobot=robotInstance.robot
           if theRobot.isAlive()==True:
              #just before the robot was alive so now we lost it :-(
              print "R%d :Robot not alive" % missingBotNR
              theRobot.setAlive(False)

    def cbWorldModelCalculated(self, data):
         try:
	   infoPage=[]
           theRobot=self.robotArray[ self.selectedRobot ].robot
           theBall=self.robotArray[ self.selectedRobot ].ball
           theObstacleList=self.robotArray[ self.selectedRobot ].obstacleList
           theObstacleListIsDirty=False

           infoPage.append("Robot: %s" % self.selectedRobot )
  
	   for event in pygame.event.get():
              keyMods=pygame.key.get_mods() 
	      keysPressed=pygame.key.get_pressed()
              #print event,  keyMods, keysPressed

              if event.type==pygame.MOUSEBUTTONUP and event.button==1: #left button
                 self.LeftMouseDown=False
              if event.type==pygame.MOUSEBUTTONDOWN and event.button==1: #left button 
                 self.LeftMouseDown=True
              if event.type == pygame.MOUSEBUTTONUP and event.button == 3:  # right button
                 self.RightMouseDown = False
              if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:  # right button 
                 self.RightMouseDown = True


	      if event.type==pygame.KEYUP:
	         #was the ? key released (help)
	         if event.key == pygame.K_0:
	            self.fakeNoBall=False  #back to normal ball info

              elif event.type==pygame.KEYDOWN:
		      if self.RightMouseDown:  #first handle all rightmousedown + key combinations
			 mouseX,mouseY=pygame.mouse.get_pos()
			 fieldCoordX,fieldCoordY=self.gameView.pixel2fieldCoord( (mouseX,mouseY-50) )  #correct for fieldwindowY offset

			 if event.key ==pygame.K_r:  #Robot key with Right Mouse is move, turn towards target pos
		            #(robotX,robotY,robotPhi)=theRobot.getCurrentLocation()
		            #hypdeg=FalconsMath.distanceCalc( robotX, robotY, fieldCoordX, fieldCoordY )
		            #theta=FalconsMath.normalizeAngle( math.radians( hypdeg[1] + 90)  )
		            commandString="action move target=coord:%0.1f,%0.1f facing=ball" % ( fieldCoordX,fieldCoordY )
		            theRobot.sendTPControlActionToRobot( commandString )
		            #self.gameView.boxText("task", [ "MOVETO %s,%s T:%0.2f"%(fieldCoordX,fieldCoordY,theta) ])

		      elif self.LeftMouseDown: #second handle all leftmousedown + key combinations
			 mouseX,mouseY=pygame.mouse.get_pos()
			 #rrint "mousedown pos: %d,%d" % ( mouseX,mouseY)
			 fieldCoordX,fieldCoordY=self.gameView.pixel2fieldCoord( (mouseX,mouseY-50) )  #correct for fieldwindowY offset
			 #print "fieldCoords: %0.2f,%0.2f" % (fieldCoordX,fieldCoordY)

			 keysPressed=pygame.key.get_pressed()
			 if event.key==pygame.K_b:  #Ball key?
		            theBall.setLocation( fieldCoordX, fieldCoordY )
			 if event.key==pygame.K_t:  #Teleport Ball key?
		            theBall.setSimLocation( fieldCoordX, fieldCoordY )
			 if event.key==pygame.K_r:  #Robot key with Left Mouse is move, keep current Phi
		            #(robotX,robotY,robotPhi)=theRobot.getCurrentLocation()
		            commandString="action move target=coord:%0.1f,%0.1f facing=robot" % ( fieldCoordX,fieldCoordY )
		            theRobot.sendTPControlActionToRobot( commandString )
		            #theRobot.setLocation( fieldCoordX, fieldCoordY, robotX, robotY ) 
		            #self.gameView.boxText("task", [ "MOVETO %s,%s keeping robot theta:%0.2f"%(fieldCoordX,fieldCoordY,robotPhi) ])

		         obstNR=-1

			 if keysPressed[pygame.K_1] or keysPressed[pygame.K_KP1]: 
			    obstNR=1
			 if keysPressed[pygame.K_2] or keysPressed[pygame.K_KP2]: 
			    obstNR=2
			 if keysPressed[pygame.K_3] or keysPressed[pygame.K_KP3]:
			    obstNR=3
			 if keysPressed[pygame.K_4] or keysPressed[pygame.K_KP4]: 
			    obstNR=4
			 if keysPressed[pygame.K_5] or keysPressed[pygame.K_KP5]: 
			    obstNR=5
			 if keysPressed[pygame.K_6] or keysPressed[pygame.K_KP6]: 
			    obstNR=6
			 if keysPressed[pygame.K_7] or keysPressed[pygame.K_KP7]: 
			    obstNR=7
			 if keysPressed[pygame.K_8] or keysPressed[pygame.K_KP8]: 
			    obstNR=8
			 if keysPressed[pygame.K_9] or keysPressed[pygame.K_KP9]: 
			    obstNR=9
			 if keysPressed[pygame.K_0] or keysPressed[pygame.K_KP0]: 
			    obstNR=0

		         if obstNR != -1:
		            try:
		               theObstacle=theObstacleList.get()[ obstNR ]
		               theObstacle.setLocation( fieldCoordX, fieldCoordY )
		               theObstacleListIsDirty=True
		            except IndexError:
		               obstNR=-1  #dont do anything, obstacle not in List

                      #possible behavior strings can be found in http://git.falcons-robocup.nl/falcons/code/blob/master/packages/teamplay/include/int/types/cDecisionTreeTypes.hpp
                      #possible action strings can be found in http://git.falcons-robocup.nl/falcons/code/blob/master/packages/teamplay/include/int/types/cActionTypes.hpp

		      elif keyMods & pygame.KMOD_CTRL and keyMods & pygame.KMOD_SHIFT and not keyMods & pygame.KMOD_ALT:    #handle control + shift + key combinations
		         if keysPressed[pygame.K_s]: 
		            theRobot.sendTPControlCommandToRobot( "stop" )
		         elif keysPressed[pygame.K_r]: 
		            theRobot.sendTPControlCommandToRobot( "reset" )

		      elif keyMods & pygame.KMOD_CTRL and keyMods & pygame.KMOD_ALT and not keyMods & pygame.KMOD_SHIFT:    #handle control + ALT + key combinations
		         if keysPressed[pygame.K_g]:  
		            theRobot.sendTPControlBehaviorToRobot( "behavior B_goalkeeper" )
		         elif keysPressed[pygame.K_d]:  
		            theRobot.sendTPControlBehaviorToRobot( "behavior defend" )

		      elif keyMods & pygame.KMOD_CTRL and not keyMods & pygame.KMOD_SHIFT and not keyMods & pygame.KMOD_ALT: #handle Control + key combinations

		         if keysPressed[pygame.K_g]: 
		            theRobot.sendTPControlActionToRobot( "behavior getBall" )
		            #self.gameView.boxText("task", [ "GOTO BALL" ])
		         elif keysPressed[pygame.K_s]:  
		            #theRobot.sendTPControlActionToRobot( "action shoot shootType=shootTowardsGoal" )
                            theRobot.sendTPControlActionToRobot( "behavior shootAtGoal" )
		         elif keysPressed[pygame.K_m]:  # 
		            theRobot.sendTPControlActionToRobot( "action moveToFreeSpot" )
		         elif keysPressed[pygame.K_i]:  # 
		            theRobot.sendTPControlActionToRobot( "action interceptBall" )

		      elif keyMods & pygame.KMOD_SHIFT and not keyMods & pygame.KMOD_CTRL and not keyMods & pygame.KMOD_ALT:    #handle Shift + key combinations
		         if keysPressed[pygame.K_b]:   #toggle own ball of/on
		            theBall.toggleFeedOwnBall()
		         if keysPressed[pygame.K_o]:   #toggle own obstacles of/on
		            theObstacleList.toggleFeedOwnObstacles()

		      else:  #normal key pressed , no shift/control/alt combinations
		         # number is select robot
		         previousSelectedRobot=self.selectedRobot
			 if keysPressed[pygame.K_1] or keysPressed[pygame.K_KP1]: 
			    self.selectedRobot='1'
			 if keysPressed[pygame.K_2] or keysPressed[pygame.K_KP2]: 
			    self.selectedRobot='2'
			 if keysPressed[pygame.K_3] or keysPressed[pygame.K_KP3]: 
			    self.selectedRobot='3'
			 if keysPressed[pygame.K_4] or keysPressed[pygame.K_KP4]: 
			    self.selectedRobot='4'
			 if keysPressed[pygame.K_5] or keysPressed[pygame.K_KP5]: 
			    self.selectedRobot='5'
			 if keysPressed[pygame.K_6] or keysPressed[pygame.K_KP6]: 
			    self.selectedRobot='6'
		         if self.selectedRobot != previousSelectedRobot:
		            #clean up old messages and other stuff
		            self.gameView.boxText("task", [ "" ])

			 if keysPressed[pygame.K_r] and keysPressed[pygame.K_o]: #o+r pressed = obstacle reset key
			    for theObstacle in theObstacleList.get():
			       theObstacle.reset()   # reset to initial values
		               theObstacleListIsDirty=True
		         elif keysPressed[pygame.K_KP_PLUS]:
		            curPower=theRobot.getShootPower()
		            if keyMods & pygame.KMOD_SHIFT:
		               curPower=curPower+10
		            else:
		               curPower=curPower+1
		            if curPower>255:
		               curPower=0
		            theRobot.setShootPower( curPower )
		         elif keysPressed[pygame.K_KP_MINUS]:
		            curPower=theRobot.getShootPower()
		            if keyMods & pygame.KMOD_SHIFT:
		               curPower=curPower-10
		            else:
		               curPower=curPower-1
		            if curPower<0:
		               curPower=255
		            theRobot.setShootPower( curPower )

			 #was the S key pressed (stop)
			 elif keysPressed[pygame.K_s]:
			    dummy=1
		            #self.thisRobot.stop()
			 #was the ? key pressed (help)
			 elif keysPressed[pygame.K_h]:
		            if self.showHelpWindow == False:
		               self.showHelpWindow=True
		            else:
		               self.showHelpWindow=False
			 #was the i key pressed (infoscreen)
			 elif keysPressed[pygame.K_i]:  #toggle info window
		            if self.showInfoWindow == False:
		               self.gameView.clearFeatures()
		               self.myPOIs.drawPOIFeature( self.gameView )
		               #self.myAreas.drawAreaFeature( self.gameView )
		               self.showInfoWindow=True
		            else:
		               self.showInfoWindow=False
			 elif keysPressed[pygame.K_f]:  #toggle feature window
		            if self.showFeatureWindow == False:
		               self.showFeatureWindow=True
		            else:
		               self.showFeatureWindow=False
			 elif keysPressed[pygame.K_0]:  
			    self.fakeNoBall=True  #fake NoBall
		         elif keysPressed[pygame.K_RIGHT] and self.showFeatureWindow==True:
		            self.gameView.clearFeatures()
		            if self.showPOIFeatures==True:
		               self.myPOIs.drawNextPOIFeature( self.gameView )
		            else:
		               self.myAreas.drawNextAreaFeature( self.gameView )
		         elif keysPressed[pygame.K_LEFT] and self.showFeatureWindow==True:
		            self.gameView.clearFeatures()
		            if self.showPOIFeatures==True:
		               self.myPOIs.drawPreviousPOIFeature( self.gameView )
		            else:
		               self.myAreas.drawPreviousAreaFeature( self.gameView )
		         elif keysPressed[pygame.K_p] and self.showFeatureWindow==True:  #switch to POI display
		            self.showPOIFeatures=True
		            self.showAreaFeatures=False
		         elif keysPressed[pygame.K_a] and self.showFeatureWindow==True:  #switch to Area display
		            self.showPOIFeatures=False
		            self.showAreaFeatures=True


	      if event.type==pygame.QUIT:
	         pygame.quit()
                 rospy.signal_shutdown("window closed")
                 quit()

	   self.gameView.clearOpp()

           if theObstacleList.autoFeedIsOn():
              obstcount=0
              obstCoordList=theObstacleList.getObstacleCoordList()

              for obstCoord in obstCoordList:
   	         self.gameView.drawOpponent(  obstCoord , "%s" %obstcount )
                 infoPage.append( "Obstacle %d: %s" % (obstcount, obstCoord) )
                 obstcount=obstcount+1
   
              if theObstacleListIsDirty:  #has anything changed? then send the obstacle list to robot
                 theObstacleList.sendObstacleListToRobot() 

	   #teammembers=getTeammembers().teammembers
           #ourteam= teammembers.RobotArray + [ self.thisRobot.getLocation().ownRobot ]
	   self.gameView.clearOwn()
	   #for teammember in ourteam:
	   #   #print "%f %f" % (teammember.Pos.x,teammember.Pos.y)
           (x,y,phi)=theRobot.getCurrentLocation()
           self.gameView.boxText("robotC", [ "Robot @ %0.1f,%0.1f,%0.1f" % (x,y,phi) ])
	   self.gameView.drawOwnRobot( (x,y), "%s" % self.selectedRobot )

	   #ballCoord=theBall.getLocation()
	   #if ballCoord[0]!=reasoning_definitions.B_ballPosNotProvided:
           
           if theBall.autoFeedIsOn():
	      self.gameView.drawBall( theBall.getLocation() ) 
           else:
              self.gameView.clearBall()

           #obstacles=getOwnObstacles()
           #obstcount=1
           #for obstacle in obstacles.obstacles.RobotArray:
           self.gameView.boxText("robotnumber", [ "Robot %s" % self.selectedRobot ])
           #self.gameView.boxText("ballOwner", [ theBall.feedStatus() ])
           self.gameView.boxText("stateinfo", [ theObstacleList.feedStatus() ])  #use the stateinfobox for display
           if theBall.autoFeedIsOn():
              self.gameView.boxText("ballC", [ "VirtBall @ %0.1f,%0.1f"  % theBall.getLocation() ])
           else:
              self.gameView.boxText("ballC", [ "VirtBallFeed=OFF" ])
           #misuse pathplanning box to show shooting power
           self.gameView.boxText("pathPlanning", [ "P:%d" % theRobot.getShootPower() ] )
           self.gameView.boxText("info", infoPage)
	   self.gameView.showHelp( self.showHelpWindow )
	   self.gameView.showInfo( self.showInfoWindow )
           self.gameView.showFieldFeatures( self.showFeatureWindow )

           if theRobot.isAlive() == False:
              self.gameView.showCross( True )
           else:
              self.gameView.showCross( False )


	   self.gameView.screenUpdate()

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



if __name__ == '__main__':  
    
        rospy.init_node('interactiveFieldSetup')


    
	teamName="teamA"
        rosNamespace="/" + teamName + "/" 

	commandChannelHandle = rospy.Publisher( rosNamespace+ 'g_commandChannelToRobots', t_commandChannel, queue_size=3)

        rosloop = rosLoop()
