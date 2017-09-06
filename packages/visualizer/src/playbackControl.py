""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 2016-07-12 JFEI creation
#
# Description: part of playbackMatchlog: little window with sliders and buttons
#
#
# 


import sys,os
import signal
import pygame
from pygame.locals import *
sys.path.append("/home/robocup/lib/pgu")
from pgu import gui


class PlaybackControl(gui.Table):
    def __init__(self, tStart, tEnd, tElapsed, **params):
        gui.Table.__init__(self, **params)
        self.speed = 1.0
        self.t = 0.0
        self.paused = False
        def cb_slider_changed(slider):
            self.t = slider.value
        def cb_faster():
            self.speed *= 2.0
        def cb_slower():
            self.speed /= 2.0
        def cb_pause():
            self.paused = True
        def cb_play():
            self.paused = False
        fg = (255,255,255)
        self.tr()
        # first row of buttons: 
        #   - pause       ||
        #   - resume      >
        #   - slow down   <<
        #   - speedup     >>
        #   - time slider 
        btn = gui.Button("||")
        btn.connect(gui.CLICK, cb_pause)
        self.td(btn)
        btn = gui.Button(">")
        btn.connect(gui.CLICK, cb_play)
        self.td(btn)
        btn = gui.Button("<<")
        btn.connect(gui.CLICK, cb_slower)
        self.td(btn)
        btn = gui.Button(">>")
        btn.connect(gui.CLICK, cb_faster)
        self.td(btn)
        self.td(gui.Label(""))
        self.td(gui.Label(""))
        self.td(gui.Label(""))
        self.timeslider = gui.HSlider(0.0, 0.0, tElapsed, size=20, width=400, height=16, name='time', colspan=15)
        self.timeslider.connect(gui.CHANGE, cb_slider_changed, self.timeslider)
        self.td(self.timeslider)
        self.app = gui.App()
        self.app.init(self)

    def run(self):
        self.app.connect(gui.QUIT,self.app.quit,None)
        self.app.run()
        
    def quit(self):
        self.app.quit()

    def updateTime(self, dt):
        if not self.paused:
            self.t += self.speed * dt
            self.timeslider.value = self.t
        return self.t


