""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
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



