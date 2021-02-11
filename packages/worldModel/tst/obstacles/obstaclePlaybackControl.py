# Copyright 2018-2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

import socket
import sys,os
import pygame
from pygame.locals import *
sys.path.append(os.path.expanduser("~/lib/pgu"))
from pgu import gui


class PlaybackControl(gui.Table):
    def __init__(self, **params):
        gui.Table.__init__(self, **params)
        self.speed = 1.0
        self.t = 0.0
        self.paused = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = 7060
        self.ip = "127.0.0.1"
        
        def cb_rewind():
            self.sock.sendto("REWIND", (self.ip, self.port))

        def cb_step_back():
            self.sock.sendto("STEP_BACK", (self.ip, self.port))

        def cb_pause():
            self.sock.sendto("PAUSE", (self.ip, self.port))

        def cb_step_forward():
            self.sock.sendto("STEP_FORWARD", (self.ip, self.port))

        def cb_play():
            self.sock.sendto("PLAY", (self.ip, self.port))

        self.tr()        

        btn = gui.Button("<<")
        btn.connect(gui.CLICK, cb_rewind)
        self.td(btn)

        btn = gui.Button("<")
        btn.connect(gui.CLICK, cb_step_back)
        self.td(btn)

        btn = gui.Button("||")
        btn.connect(gui.CLICK, cb_pause)
        self.td(btn)

        btn = gui.Button(">")
        btn.connect(gui.CLICK, cb_step_forward)
        self.td(btn)
        
        btn = gui.Button(">>")
        btn.connect(gui.CLICK, cb_play)
        self.td(btn)
        
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

pbCtrl = PlaybackControl()
pbCtrl.run()

