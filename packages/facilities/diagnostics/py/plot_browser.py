""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python
# create a window, setup key bindings to browse through an array of Plots
# a Plot consists of a set of curves and labels, each with fully customizable formatting
# intended to plot intercept events and shoot/pass events


import matplotlib.pyplot as plt
import matplotlib as mpl
import math



class Plot():
    """
    Store a set of curves and labels which can be plotted. Allow any formatting arguments.
    """
    def __init__(self):
        self.curves = []
        self.labels = []
        self.callback = None
    def addCurve(self, x, y, name, *args, **kwargs):
        self.curves.append({'x': x, 'y': y, 'name': name, 'args': args, 'kwargs': kwargs})
    def addLabel(self, x, y, text, *args, **kwargs):
        self.labels.append({'x': x, 'y': y, 'text': text, 'args': args, 'kwargs': kwargs})
    def __str__(self):
        curvesStr = ",".join(str(c['name']) for c in self.curves)
        return "Plot(curves={})".format(curvesStr)
    def plot(self, ax, clear=True, legendLoc=None):
        #print("%s clear=%r" % (str(self), clear))
        if clear:
            ax.clear()
        for curve in self.curves:
            #print("PLOT %s" % (str(curve)))
            ax.plot(curve['x'], curve['y'], label=curve['name'], *curve['args'], **curve['kwargs'])
        for label in self.labels:
            ax.text(label['x'], label['y'], label['text'], *label['args'], **label['kwargs'])
        ax.legend(loc=legendLoc)
        ax.grid(True)
        if self.callback:
            self.callback(ax)


class PlotBrowser():
    """
    Construct a window and browse through plots. Setup key handler.
    """
    def __init__(self, title='plot browser'):
        self.plots = []
        self.currentIdx = 0
        self.zoomMargin = 0.08
        self.defaultXlim = None
        self.defaultYlim = None
        self.defaultLegendLoc = None
        self.title = title
        self.connections = [] # other instances
        self.constructFigure()

    def constructFigure(self):
        # setup the figure and connect key bindings
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.set_window_title(self.title)
        self.fig.canvas.mpl_connect('key_press_event', self.keyCallback)
        # disable conflicting standard bindings
        if 'left' in mpl.rcParams['keymap.back']:
            mpl.rcParams['keymap.back'].remove('left')
        if 'right' in mpl.rcParams['keymap.forward']:
            mpl.rcParams['keymap.forward'].remove('right')
        if 'f' in mpl.rcParams['keymap.fullscreen']:
            mpl.rcParams['keymap.fullscreen'].remove('f')
        if 'l' in mpl.rcParams['keymap.yscale']:
            mpl.rcParams['keymap.yscale'].remove('l')

    def keyCallback(self, event):
        # note: most standard keyboard shortcuts from matplotlib toolbar also work, like h/r to reset zoom
        if event.key in ['n', 'right']: # next, allow wrap around
            self.select(self.currentIdx + 1)
        elif event.key in ['p', 'left']: # previous, allow wrap around
            self.select(self.currentIdx - 1)
        elif event.key in ['pageup']:
            self.select(min(len(self.plots) - 1, self.currentIdx + 5)) # do not wrap around
        elif event.key in ['pagedown']:
            self.select(max(0, self.currentIdx - 5)) # do not wrap around
        elif event.key in ['f']: # first
            self.select(0)
        elif event.key in ['l']: # last
            self.select(-1)
        elif event.key in ['q']: # quit
            plt.close('all')
        elif event.key in ['m']: # full-screen toggle
            mng = plt.get_current_fig_manager()
            mng.full_screen_toggle()

    def select(self, idx, followConnections=True):
        self.currentIdx = idx % len(self.plots) # ensure a valid index
        self.plots[self.currentIdx].plot(self.ax, legendLoc=self.defaultLegendLoc)
        if followConnections:
            for other in self.connections:
                other.select(idx, followConnections=False)
        self.autoZoom()

    def autoZoom(self):
        if self.defaultXlim:
            self.ax.set_xlim(self.defaultXlim)
        if self.defaultYlim:
            self.ax.set_ylim(self.defaultYlim)
        self.fig.canvas.draw()

    def connect(self, other):
        """
        Connect an instance with another, synchronizing browsing. Assumes equal length.
        """
        # bidirectional
        self.connections.append(other)
        other.connections.append(self)
        
    def show(self):
        plt.show()


if __name__ == "__main__":

    # EXAMPLE

    b = PlotBrowser('window1')
    b2 = PlotBrowser('window2')
    b2.connect(b)

    # plot 1
    p = Plot()
    t = range(20)
    p.addCurve(t, [math.sin(x) for x in t], 'sin')
    p.addCurve(t, [x*x*1e-2 for x in t], 'sqr')
    p.addLabel(10, 1, 'hoi')
    b.plots.append(p)
    b2.plots.append(p)

    # plot 2
    # arguments are allowed, just passed down to plot() and text()
    p = Plot()
    p.addCurve(1, 1, 'empty', '*')
    p.addLabel(0, 0, 'empty', color='red', backgroundcolor='grey')
    b.plots.append(p)
    b2.plots.append(p)

    # show
    b.select(1)
    b.show()

