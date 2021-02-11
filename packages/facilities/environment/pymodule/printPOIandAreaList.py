# Copyright 2016-2020 Michel Koenen (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
import EnvironmentField
from EnvironmentField import *

myField=EnvironmentField.cEnvironmentField.getInstance()
#myPOI=EnvironmentField.poiInfo()     #preprare myPOI structure
#myArea=EnvironmentField.areaInfo()

def printPOI( thePOI ):
   print "id: %s" % thePOI.id
   print "x:  %0.2f" % thePOI.x
   print "y:  %0.2f" % thePOI.y

def getPOI(poistring):
   myPOI=EnvironmentField.poiInfo()     #preprare myPOI structure
   myField.getFieldPOIByString( poistring, myPOI)
   printPOI( myPOI) 

def getArea(areaString):
   myArea=EnvironmentField.areaInfo() #prep myArea structure
   myField.getFieldAreaByString( areaString, myArea)
   print "id: %s" % myArea.id
   print "type: %c" % myArea.typeC
   if myArea.typeC == 'R':  # rectangle
      print "Rectangle corners:"
      getPOI( myArea.R.corner1.id )
      getPOI( myArea.R.corner2.id )
      print "MinX=%0.2f, MaxX=%0.2f, MinY=%0.2f, MaxY=%0.2f" % (myArea.R.getMinX(), myArea.R.getMaxY(), myArea.R.getMinY(), myArea.R.getMaxY() )
   elif myArea.typeC == 'C':  #circle
      getPOI( myArea.C.center.id )
      print "Radius: %f" % myArea.C.radius
   elif myArea.typeC == 'T': #triangle
      printPOI( myArea.T.corner1 )
      printPOI( myArea.T.corner2 )
      printPOI( myArea.T.corner3 )

   elif myArea.typeC == 'S': #semicircle
      getPOI( myArea.S.center.id )
      print "Radius: %f" % myArea.S.radius
      print "BaseAngle: %f" % myArea.S.baseAngle
   else:
      print "ERROR, unkown area type received"


print "Field width: " , myField.getWidth()
print "Field length: " , myField.getLength()

poiList=StringVector()
myField.getFieldPOIList( poiList )
print "Number of POIs: ", poiList.size()
for i in poiList:
   getPOI( i )
   print

#getPOI("P_CENTER")

areaList=StringVector()
myField.getFieldAreaList( areaList )
print "Number of Areas: ", areaList.size()
for i in areaList:
   getArea( i )
   print



