# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle



class Checkbox():
    def __init__(self, name, position, default=False, label=None, rsize=0.6, enabled=True):
        self.name = name # unique ID associated with
         # label to display next to the checkbox
        if label == None:
            self.label = name # reuse
        else:
            self.label = label
        self.callback = None
        self.enabled = enabled
        self.ticked = default
        self.ax = plt.axes(position) # position is a tuple (x,y,w,h)
        self.ax.axis('off')
        self.canvas = self.ax.figure.canvas
        # draw text
        if len(self.label):
            self.text = self.ax.text(-0.15, 0.5, self.label, horizontalalignment='right', verticalalignment='center')
        # draw a rectangle, add a bit of spacing
        self.ax.add_patch(Rectangle((0,(1.0-rsize)/2), rsize, rsize, fill=True))
        # setup event handling
        self.canvas.mpl_connect('button_release_event', self._handle_event)
        self.redraw()

    def __repr__(self):
        s = 'checkbox:' + self.name + '=' + str(self.ticked)
        if not self.enabled:
            s += ' (disabled)'
        return s

    def on_changed(self, cb):
        self.callback = cb

    def _handle_event(self, e):
        if self.enabled and e.inaxes == self.ax: # TODO: exclude spacing margin for inaxes calculation
            self.ticked = not self.ticked
            self.redraw()
            if self.callback != None:
                self.callback(self.name, self.ticked)

    def redraw(self):
        col = 'grey'
        if self.enabled:
            col = ['lightgoldenrodyellow', 'blue'][self.ticked]
        self.ax.patches[0].set_facecolor(col)
        self.ax.figure.canvas.draw()

