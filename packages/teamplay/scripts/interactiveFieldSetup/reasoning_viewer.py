""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # History:
# v 0.1 MKOE 20150301 first release to repository
# v 0.2 MKOE 20150315 addition of info & help windows
# v 0.3 MKOE 20150402 show robotnumber
# v 0.4 MKOE 20150408 show self set opponent beta
#                     added extra info row
#                     enlarged task display
# v 0.5 MKOE 20160302 impl. field mirror for other team display
# v 0.6 MKOE 20160321 small modifications to work with interactiveFieldSetup.py
# v 0.7 MKOE 20160424 expand size of help window

# Description: graphical module for reasoning, will display field POIs, Areas, role/task etc.
#              makes use of pygame package
#
# usage:  import <this module>
#
# 
import sys,os
import pygame

try:
   import reasoning_definitions
except:
   my_python_library_files= os.path.dirname(os.path.realpath(__file__)) + "/reasoning"
   sys.path.append( my_python_library_files )
   import reasoning_definitions

class GraphField:
   def __init__(self, windowTitle, ourColor, mirror):

      self.GREEN  = (  0, 200,   0)
      self.WHITE  = (255, 255, 255)
      self.RED    = (255,   0,   0)
      self.YELLOW = (255, 255,   0)
      self.GRAY   = (100, 100, 100)
      self.BLACK  = (  1,   1,   1)
      self.BLUE   = (  0,   0, 255)
      self.MAGENTA= (255,   0, 255)
      self.CYAN   = (  0, 255, 255)


      self.windowW=640
      self.windowH=480
      self.zoom=25
      if mirror == False :
         self.mirror=1
      else:
         self.mirror=-1

      self.showHelpWindow=False
      self.showInfoWindow=False
      self.showCrossWindow=False
      self.showFieldFeatureWindow=False

      if ourColor == 'cyan' : 
         self.ourColor= self.CYAN
         self.oppColor= self.MAGENTA
      else:
         self.ourColor= self.MAGENTA
         self.oppColor= self.CYAN

      self.lineThickness=int( reasoning_definitions.F_lineThickness * self.zoom )

      pygame.init()
      pygame.display.set_caption(windowTitle)   # set window title
      self.screen=pygame.display.set_mode( [self.windowW, self.windowH] )
      screenDimensions=self.screen.get_rect()
      self.screenCenter=( screenDimensions.centerx, screenDimensions.centery )

      self.backgroundWindow=pygame.Surface( self.screen.get_size() )
      self.backgroundWindow.fill( self.GREEN )

      self.fieldWindow=pygame.Surface( (self.windowW-50, self.windowH-50) )
      self.fieldWindow.fill( self.GREEN )           # football field color
      #self.fieldWindow.set_alpha(100)
      fieldWindowDimensions= self.fieldWindow.get_rect()

      self.fieldFeatureWindow=pygame.Surface( self.fieldWindow.get_size() )
      self.fieldFeatureWindow.set_colorkey( (0,0,0) )

      self.botWindow=pygame.Surface( self.fieldWindow.get_size() )
      self.botWindow.set_colorkey( (0,0,0) )

      self.opponentWindow=pygame.Surface( self.fieldWindow.get_size() )
      self.opponentWindow.set_colorkey( (0,0,0) )

      self.ballWindow=pygame.Surface( self.fieldWindow.get_size() )
      self.ballWindow.set_colorkey( (0,0,0) )

      self.crossWindow=pygame.Surface( self.fieldWindow.get_size() )
      self.crossWindow.set_colorkey( (0,0,0) )

      #opponent font
      self.oppFont = pygame.font.Font(None, 25 )
      self.ownFont = pygame.font.Font(None, 25 )

      self.centerXoffset=fieldWindowDimensions.centerx
      self.centerYoffset=fieldWindowDimensions.centery

      self.statusWindow=pygame.Surface( ( self.windowW-100, 65) )
      self.statusWindow.fill( self.GRAY )

      self.helpWindow=pygame.Surface( ( 600,350 ) )
      self.helpWindow.fill( self.GRAY )
      self.helpWindow.set_alpha(200)

      self.infoWindow=pygame.Surface( ( 600,350 ) )
      self.infoWindow.fill( self.GRAY )
      self.infoWindow.set_alpha(200)

      this_folder= os.path.dirname(os.path.realpath(__file__)) 
      self.logo=pygame.image.load("%s/falcons_logo.png" % this_folder)
      self.logo=aspect_scale( self.logo, (80,120) )

      #TextBox params: name, myWidth, myHeight, fontColor, fontSize, bgColor
      self.gameStateInfoBox=TextBox("stateinfo", 200, 18, self.BLACK, 20, self.WHITE)
      self.stateBox=TextBox("state", 150, 18, self.BLACK, 20, self.WHITE )
      self.roleBox= TextBox("role",  150, 18, self.BLACK, 20, self.WHITE )
      self.robotCoordBox=TextBox("robotC", 150, 18, self.BLACK, 22, self.WHITE )
      self.ballCoordBox=TextBox("ballC", 150, 18, self.BLACK, 22, self.WHITE )
      self.ballOwnerBox=TextBox("ballOwner", 160, 18, self.BLACK, 22, self.WHITE )
      self.pathplanningBox=TextBox("pathPlanning", 50, 18, self.BLACK, 22, self.WHITE )
      self.taskBox=TextBox("task", 370, 18 , self.BLACK, 22, self.WHITE )
      self.helpBox=TextBox("help", self.windowW-20 ,self.windowW-30, self.BLACK, 22, self.GRAY) 
      self.featureNameBox=TextBox("featureName", 490, 18, self.BLACK, 22, self.WHITE) 
      self.infoBox=TextBox("info", self.windowW-20 ,self.windowW-30, self.BLACK, 15, self.GRAY) 
      self.robotNumberBox=TextBox("robotnumber", 110,20, self.BLACK, 22, self.GRAY) 
      self.screenUpdate()

   def events(self):
         return pygame.event.get()

   def screenUpdate(self):
      #build up screen pixels in correct order
      #self.screen.blit( self.bgwin, ( (self.screenCenter[0]-self.bgCenter[0]), (self.screenCenter[0]-self.bgCenter[1]) ))
      self.screen.blit( self.backgroundWindow, (0,0) )
      self.screen.blit( self.fieldWindow, (0,50) )
      self.screen.blit( self.botWindow, (0,50) )
      self.screen.blit( self.opponentWindow, (0,50) )
      self.screen.blit( self.ballWindow, (0,50) )

      statusWindowPos=self.statusWindow.get_rect()
      statusWindowPos.x=5
      statusWindowPos.y=5
      self.statusWindow.fill( (100,100,100) )  #clear..
      self.statusWindow.blit( self.stateBox.windowHandle(), (3,3) )
      self.statusWindow.blit( self.ballCoordBox.windowHandle(), (3+150+5,3) ) 
      self.statusWindow.blit( self.ballOwnerBox.windowHandle(), (3+150+5+150+5,3) )   
      self.statusWindow.blit( self.pathplanningBox.windowHandle(), (3+150+5+150+5+160+5,3) ) 

      self.statusWindow.blit( self.roleBox.windowHandle(), (3,23) )  
      self.statusWindow.blit( self.robotCoordBox.windowHandle(), (3+150+5,23) )
      self.statusWindow.blit( self.gameStateInfoBox.windowHandle(), (3+150+5+150+5,23) ) 
 
      self.statusWindow.blit( self.taskBox.windowHandle(), (3,43) ) 

      self.screen.blit( self.statusWindow, statusWindowPos )
      self.screen.blit( self.logo, ( self.windowW-80,0) )
      self.screen.blit( self.robotNumberBox.windowHandle(), (self.windowW-110, self.windowH-20)  )

      if self.showHelpWindow:
         helpWindowPos=self.helpWindow.get_rect()
         #helpWindowPos.centerx=self.centerXoffset
         #helpWindowPos.centery=self.centerYoffset
         helpWindowPos.x=10
         helpWindowPos.y=80
         self.helpWindow.blit( self.helpBox.windowHandle(), (5,5) )
         self.screen.blit( self.helpWindow, helpWindowPos)

      if self.showInfoWindow:
         infoWindowPos=self.infoWindow.get_rect()
         infoWindowPos.x=10
         infoWindowPos.y=80
         self.infoWindow.blit( self.infoBox.windowHandle(), (5,5) )
         self.screen.blit( self.infoWindow, infoWindowPos)

      if self.showCrossWindow:
         self.screen.blit( self.crossWindow, (0,50) )

      if self.showFieldFeatureWindow  :
         featureWindowPos=self.fieldFeatureWindow.get_rect()
         self.fieldFeatureWindow.blit( self.featureNameBox.windowHandle(), (50,25) )
         self.screen.blit( self.fieldFeatureWindow, (0,50))


      pygame.display.flip()

   def field2pixelCoord( self, xytuple ):
      #robot field X = coord[0] = Ycoord on graphic
      #robot field Y = coord[1] = Xcoord on graphic
      fieldpixelX=self.centerXoffset + self.mirror * (int( self.zoom * xytuple[1] ))
      fieldpixelY=self.centerYoffset + self.mirror * (int( self.zoom * xytuple[0] ))
      return ( fieldpixelX, fieldpixelY )

   def pixel2fieldCoord( self, xypixel ):
      fieldY=((xypixel[0]-self.centerXoffset)*1.0 / self.zoom) * self.mirror
      fieldX=((xypixel[1]-self.centerYoffset)*1.0 / self.zoom) * self.mirror
      return ( fieldX,fieldY )

   def clearOwn(self):
      self.botWindow.fill( (0,0,0) )  # clear

   def drawOwnRobot(self, coord, number):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]
      pygame.draw.circle(self.botWindow, self.ourColor , [x,y], 
                        int(reasoning_definitions.R_botRadius*self.zoom),0)
      robotNumberText = self.ownFont.render(number, 1, self.BLACK )
      self.botWindow.blit( robotNumberText, (x-5,y-10) )

   def drawBot(self, coord):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]
      self.botWindow.fill( (0,0,0) )  # clear
      pygame.draw.circle(self.botWindow, self.ourColor , [x,y], 
                        int(reasoning_definitions.R_botRadius*self.zoom),0)

   def clearOpp(self):
      self.opponentWindow.fill( (0,0,0) )  # clear

   def drawOpponent(self, coord, number):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]
      pygame.draw.circle(self.opponentWindow, self.oppColor , [x,y], 
                        int(reasoning_definitions.R_botRadius*self.zoom),0)
      oppNumberText = self.oppFont.render(number, 1, self.BLACK )
      self.opponentWindow.blit( oppNumberText, (x-5,y-10) )

   #old function keep for compatibility reasons
   def drawOpp(self, coord):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]
      self.opponentWindow.fill( (0,0,0) )  # clear
      pygame.draw.circle(self.opponentWindow, self.oppColor , [x,y], 
                        int(reasoning_definitions.R_botRadius*self.zoom),0)
   
   def clearBall(self):
      self.ballWindow.fill( (0,0,0) )  # clear

   def clearFeatures(self):
      self.fieldFeatureWindow.fill( (0,0,0) )  #clear

   def drawBall(self, coord):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]
      self.ballWindow.fill( (0,0,0) )  # clear
      pygame.draw.circle(self.ballWindow, self.YELLOW , [x,y], 
                        int(reasoning_definitions.B_ballRadius*self.zoom),0)

   def drawFieldPoint(self, coord, windowName):
      point=self.field2pixelCoord( coord )
      x=point[0]
      y=point[1]

      if windowName == 'field':
         pygame.draw.circle(self.fieldWindow, self.WHITE, [x,y] ,self.lineThickness)
      if windowName == 'feature':
         pygame.draw.circle(self.fieldFeatureWindow, self.BLUE, [x,y] ,self.lineThickness)

   def drawFieldRectangle(self, coord1 , coord2, windowName):
      corner1=self.field2pixelCoord( coord1 )
      corner2=self.field2pixelCoord( coord2 )
      x1=corner1[0]
      y1=corner1[1]
      x2=corner2[0]
      y2=corner2[1]

      #find top left coordinate..
      topLeftX=min(x1,x2)
      topLeftY=min(y1,y2)
      width=abs(x1-x2)
      height=abs(y1-y2)
      #print topLeftX, topLeftY, width, height
      if windowName == 'field':
         pygame.draw.rect(self.fieldWindow, self.WHITE,[topLeftX, topLeftY, width, height] ,self.lineThickness)
      if windowName == 'feature':
         pygame.draw.rect(self.fieldFeatureWindow, self.BLUE, [topLeftX, topLeftY, width, height] ,self.lineThickness)


   def drawLine(self, coord1, coord2, color, windowName):
      if windowName == "cross":
        targetWindow=self.crossWindow

      point1=self.field2pixelCoord( coord1 )
      point2=self.field2pixelCoord( coord2 )
      pygame.draw.line( targetWindow, color, [point1[0], point1[1]], [point2[0], point2[1]], self.lineThickness)

   def drawFieldCircle(self, coord, radius, windowName):
      center=self.field2pixelCoord( coord )
      x=center[0]
      y=center[1]
      if windowName == 'field':
         pygame.draw.circle(self.fieldWindow, self.WHITE, [x,y] , int(self.zoom*radius), self.lineThickness)
      if windowName == 'feature':

         pygame.draw.circle(self.fieldFeatureWindow, self.BLUE, [x,y] , int(self.zoom*radius), self.lineThickness)

   def drawFieldPoly(self, coordList, windowName):
      polyList=[]
      for coord in coordList:
         polyList.append( self.field2pixelCoord( coord ) )
      # draw closed polyline
      if windowName == 'field':
         pygame.draw.lines(self.fieldWindow, self.WHITE, True, polyList, self.lineThickness )
      if windowName == 'feature':
         pygame.draw.lines(self.fieldFeatureWindow, self.BLUE, True, polyList, self.lineThickness )


   def boxText( self, boxName, theText):
      box=TextBox.myList[ boxName ]
      box.write( theText )

   def showHelp( self, value ):
      self.showHelpWindow=value

   def showInfo( self, value ):
      self.showInfoWindow=value

   def showCross( self, value ):
      self.showCrossWindow=value

   def showFieldFeatures( self, value ):
      self.showFieldFeatureWindow=value

class TextBox:
   myList={}

   def __init__(self, name, myWidth, myHeight, fontColor, fontSize, bgColor):
      self.myWidth=myWidth
      self.myHeight=myHeight
      self.mybgColor=bgColor
      self.myFontColor=fontColor
      self.myFontSize=fontSize
      self.textBoxWindow=pygame.Surface( ( self.myWidth, self.myHeight ) )   #create mini window for this box
      #self.textBoxWindow.set_colorkey( self.mybgColor )
      self.textBoxWindow.fill( self.mybgColor )
      TextBox.myList[ name ]= self

   def write( self, theText):  #write text
      font = pygame.font.Font(None, self.myFontSize )
      lineSize=font.get_linesize()
      lineYcoord=0
      self.textBoxWindow.fill( self.mybgColor )     #first clear to remove old text
      for textLine in theText:
         text = font.render(textLine, 1, self.myFontColor )
         #textRect=text.get_rect()

         self.textBoxWindow.blit( text, (0,lineYcoord) )
         lineYcoord=lineYcoord + lineSize


   def windowHandle(self):
      return self.textBoxWindow


"""
aspect_scale.py - Scaling surfaces keeping their aspect ratio
Raiser, Frank - Sep 6, 2k++
crashchaos at gmx.net

This is a pretty simple and basic function that is a kind of
enhancement to pygame.transform.scale. It scales a surface
(using pygame.transform.scale) but keeps the surface's aspect
ratio intact. So you will not get distorted images after scaling.
A pretty basic functionality indeed but also a pretty useful one.

Usage:
is straightforward.. just create your surface and pass it as
first parameter. Then pass the width and height of the box to
which size your surface shall be scaled as a tuple in the second
parameter. The aspect_scale method will then return you the scaled
surface (which does not neccessarily have the size of the specified
box of course)

Dependency:
a pygame version supporting pygame.transform (pygame-1.1+)
"""

def aspect_scale(img,(bx,by)):
    """ Scales 'img' to fit into box bx/by.
     This method will retain the original image's aspect ratio """
    ix,iy = img.get_size()
    if ix > iy:
        # fit to width
        scale_factor = bx/float(ix)
        sy = scale_factor * iy
        if sy > by:
            scale_factor = by/float(iy)
            sx = scale_factor * ix
            sy = by
        else:
            sx = bx
    else:
        # fit to height
        scale_factor = by/float(iy)
        sx = scale_factor * ix
        if sx > bx:
            scale_factor = bx/float(ix)
            sx = bx
            sy = scale_factor * iy
        else:
            sy = by

    sx=int(sx)
    sy=int(sy)

    return pygame.transform.scale(img, (sx,sy))
