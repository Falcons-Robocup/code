# Copyright 2020-2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
class RtDBPanelController():
    def __init__(self, max_vpos = None, max_hpos = None):
        self.vpos = 0
        self.hpos = 0
        self.overflow_vpos = 0
        self.max_vpos = max_vpos
        self.max_hpos = max_hpos
        self.window_max_vpos = max_hpos


    def reset(self):
        self.vpos = 0
        self.hpos = 0

    def increment_vertical(self, page = 1):
        if self.max_vpos is None:
            return

        self.vpos = min(self.max_vpos - 1, self.vpos + page)

        if self.window_max_vpos and self.vpos > self.window_max_vpos:
            difference = max(self.max_vpos - self.window_max_vpos - 1, 0)
            self.overflow_vpos = min(difference, self.overflow_vpos + page)
            self.vpos = self.window_max_vpos


    def decrement_vertical(self, page = 1):
        if self.vpos == 0:
            self.overflow_vpos = max(0, self.overflow_vpos - page)
        else:
            self.vpos = max(0, self.vpos - page)


    def increment_horizontal(self, page = 1):
        if self.max_hpos is not None:
            self.hpos = self.hpos + page if self.hpos + page < self.max_hpos else self.max_hpos - 1
    def decrement_horizontal(self, page = 1):
        self.hpos = 0 if self.hpos - page < 0 else self.hpos - page
