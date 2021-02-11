# Copyright 2015-2020 Michel Koenen (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
# Example python code to use the c-library for robocub EnvironmentField
# Assumes that the _EnvironmentField.so library is found somewhere in PYTHONPATH
# _EnvironmentField.so is build with SWIG from the cEnvironment*cpp en *hpp files
#
# 20151228 Michel Koenen
# 20160625 MKOE added also python testers for getFieldPOIByString and getFieldAreaByString
#

import EnvironmentField, EnvironmentBall, EnvironmentRobot

myField=EnvironmentField.cEnvironmentField.getInstance()
myBall=EnvironmentBall.cEnvironmentBall.getInstance()
myRobot=EnvironmentRobot.cEnvironmentRobot.getInstance()

myPOI=EnvironmentField.poiInfo()     #preprare myPOI structure
myArea=EnvironmentField.areaInfo()

print "==========================================================="
print "========                FIELD                   ==========="
print "==========================================================="
print "field Width x Length= %0.2f x %0.2f" % (myField.getWidth(), myField.getLength() )
print "getfieldPOI test:"
myField.getFieldPOI( EnvironmentField.P_OWN_GOALAREA_CORNER_LEFT , myPOI)
print "id: %s" % myPOI.id
print "x:  %0.2f" % myPOI.x
print "y:  %0.2f" % myPOI.y

print "getfieldPOIByString test:"
myPOIString="P_OPP_GOALPOST_LEFT"
myField.getFieldPOIByString( myPOIString , myPOI )
print "id: %s" % myPOI.id
print "x:  %0.2f" % myPOI.x
print "y:  %0.2f" % myPOI.y

print ""
print "getfieldArea test:"
myField.getFieldArea( EnvironmentField.A_OPP_DEFENSE_RIGHT , myArea)
print "id: %s" % myArea.id
print "type: %s" % myArea.type
print "typeC: %s" % myArea.typeC
print "R-corner1.id: %s x,y= %0.2f,%0.2f" % (myArea.R.corner1.id,myArea.R.corner1.x,myArea.R.corner1.y)
print "R-corner2.id: %s x,y= %0.2f,%0.2f" % (myArea.R.corner2.id,myArea.R.corner2.x,myArea.R.corner2.y)

print ""
print "getfieldAreaByString test:"
myAreaString="A_OWN_DEFENSE"
myField.getFieldAreaByString( myAreaString , myArea)
print "id: %s" % myArea.id
print "type: %s" % myArea.type
print "typeC: %s" % myArea.typeC
print "R-corner1.id: %s x,y= %0.2f,%0.2f" % (myArea.R.corner1.id,myArea.R.corner1.x,myArea.R.corner1.y)
print "R-corner2.id: %s x,y= %0.2f,%0.2f" % (myArea.R.corner2.id,myArea.R.corner2.x,myArea.R.corner2.y)
print ""
print "check if 0.1, 0.1 is in center circle:"
print myField.isPositionInArea( 0.1, 0.1, EnvironmentField.A_CENTER_CIRCLE )
print ""
print "check if 3.25, -6.75 margin 0.1 is in own penalty area:"
print myField.isPositionInArea( 3.25, -6.75, EnvironmentField.A_OWN_PENALTYAREA , 0.1)
print ""
print "check if 3.26, -6,74 margin 0.005 is in own penalty area:"
print myField.isPositionInArea( 3.26, -6.74, EnvironmentField.A_OWN_PENALTYAREA , 0.005)
print ""
print "lineThickness:  %0.2f" % myField.getLineThickness()
print "goalPostWidth:  %0.2f" % myField.getGoalPostWidth()
print "goalDepth:  %0.2f" % myField.getGoalDepth()
print "goalHeight:  %0.2f" % myField.getGoalHeight()

print "==========================================================="
print "========                 BALL                   ==========="
print "==========================================================="
print "ball radius: %0.2f " % myBall.getRadius()

print "==========================================================="
print "========                 ROBOT                  ==========="
print "==========================================================="
print "robot radius: %0.2f " % myRobot.getRadius()
print "robot radiusMargin: %0.2f " % myRobot.getRadiusMargin()


