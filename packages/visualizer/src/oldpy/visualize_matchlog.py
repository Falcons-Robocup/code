""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 2015-03-22 JFEI creation, borrowed part of implementation from MKOE
#
# Description: visualize matchlog topic realtime or post-match via .bag file
#
#
# 


import sys,os
import pygame
import argparse
from pygame.locals import *
sys.path.append("/home/robocup/lib/pgu")
from pgu import gui
from visualizer import Visualizer
from matchlog import MatchLogRealTime, MatchLogBagged





if __name__ == '__main__':
   # Argument parsing.
   parser     = argparse.ArgumentParser(description='falcons /g_matchlog viewer')
   parser.add_argument('bagfile', help='bag file of the log', nargs='?', default=None)
   parser.add_argument('--zoom', help='zoom factor for screen', default='auto')
   args       = parser.parse_args()
   # construct MatchLog object, either realtime or based on .bag file, and pass it
   if args.bagfile != None:
      Visualizer(MatchLogBagged(args.bagfile), True, args.zoom).run()
   else:
      Visualizer(MatchLogRealTime(), False, args.zoom).run()


