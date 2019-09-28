#!/usr/bin/env python
#
# FALCONS // Jan Feitsma, January 2018


from tracePlotBase import *


class tracePlotBall(tracePlotBase):

    def __init__(self, settings):
        tracePlotBase.__init__(self, settings)
        # require single-robot data
        if len(settings.robots) != 1:
            raise Exception("require single-robot data")
        self.robot = self.robots[self.settings.robots[0]]
        # plot zoom / selectors
        self.timeRange = None
        self.timeRangeFull = None
        self.timeMarker = None
        self.timeMarkerLine = None
        self.clickDownX = None
        self.sceneHistory = 0.1
        self.sceneHeight = 2.0
        self.sceneWidth = 2.0
        
    def reset(self):
        self.timeRange = self.timeRangeFull
        self.timeMarker = None
        self.refresh()
                
    def refresh(self):
        self.axes[0].set_xlim(self.timeRange)
        if self.timeMarkerLine != None:
            self.timeMarkerLine[0].remove()
        self.timeMarkerLine = self.axes[0].plot([self.timeMarker, self.timeMarker], [-30, 30], '-.', c='k')
        self.plotTrackers()
        self.figure.canvas.draw() # costly
                
    def keyPress(self, event):
        if event.key == 'q':
            plt.close('all')
        if event.key == 'r':
            self.reset()
        self.refresh()
        
    def onClick(self, event):
        if event.button == 1:
            # check if in top subplot
            xlim = self.axes[0].get_xlim()
            if (event.xdata >= xlim[0]) and (event.xdata <= xlim[1]):
                self.clickDownX = event.xdata

    def onRelease(self, event):
        if event.button == 1:
            # check if in top subplot
            xlim = self.axes[0].get_xlim()
            if (event.xdata >= xlim[0]) and (event.xdata <= xlim[1]):
                # either placing a marker, or zooming a range
                if event.xdata - self.clickDownX < 1e-5:
                    self.timeMarker = event.xdata
                else:
                    self.timeRange = [self.clickDownX, event.xdata]
                self.refresh()

    def plotTrackers(self):
        self.axes[1].clear()
        # plot trackers and history
        if self.timeMarker == None:
            return # nothing to be plotted
        print "marker", self.timeMarker, self.timeRange
        t = toTimeStamp(matplotlib.dates.num2date(self.timeMarker)) - EPOCH - 3600 # TODO timezone magic
        timeRange = [t - self.sceneHistory, t]
        print "timeRange", timeRange, " t=", t
        br = self.robot.br.get(timeRange)
        f = 1
        if len(br.t):
            x = br.x
            y = br.y
            z = br.z
            vx = br.vx * f
            vy = br.vy * f
            vz = br.vz * f
            print "hoi", x, y, vx, vy
            self.axes[1].quiver(x, y, z, vx, vy, vz)
        self.axes[1].set_xlim3d(x[-1] - self.sceneWidth, x[-1] + self.sceneWidth)
        self.axes[1].set_ylim3d(y[-1] - self.sceneWidth, y[-1] + self.sceneWidth)
        self.axes[1].set_zlim3d(0, self.sceneHeight)
        self.axes[1].view_init(elev=90, azim=0)
    
    def plotTimelineBR(self):
        # plot timeline
        dims = ['x', 'y', 'z', 'vx', 'vy', 'vz']
        colors = {'x': 'b', 'y': 'm', 'z': 'k', 'vx': 'b', 'vy': 'm', 'vz': 'k'}
        styles = {'x': '-', 'y': '-', 'z': '-', 'vx': '-.', 'vy': '-.', 'vz': '-.'}
        if self.timeRange == None:
            br = self.robot.br.get()
            self.timeRange = [br.t[0], br.t[-1]]
            self.timeRangeFull = self.timeRange
        else:
            br = self.robot.br.get(self.timeRange)
        timeRange = self.timeRange
        addBackgroundBlocks(self.axes[0], br.t, [n == 0 for n in br.nT], -30, 60, "#ff9999", 'lostBall')
        span = timeRange[1] - timeRange[0]
        timeRange[0] = timeRange[0] - 0.22 * span
        timeRange[1] = timeRange[1] + 0.02 * span
        self.axes[0].set_xlim(*timeRange)
        self.axes[0].set_ylim(-20, 20)
        for k in dims:
            v = getattr(br, k)
            self.axes[0].plot(br.t, v, styles[k], c=colors[k], label=k)
        # legend
        self.axes[0].legend(loc=2)
        # TODO zero axis / grid
        # make pretty time labels
        # TODO: if timerange is small enough, go to millisecond resolution
        self.figure.autofmt_xdate()
        locator = matplotlib.dates.AutoDateLocator(minticks=3)
        formatter = matplotlib.dates.DateFormatter('%H:%M:%S')
        self.axes[0].xaxis.set_major_locator(locator)
        self.axes[0].xaxis.set_major_formatter(formatter)

    def plot(self):
        # setup figure layout
        self.figure, ax = plt.subplots()
        self.axes = []
        self.axes.append(plt.subplot(211))
        self.figure.canvas.set_window_title('Harry Plotter v2.0: r%d ball tracking' % self.settings.robots[0])
        # plot top timeline
        self.plotTimelineBR()
        # now setup other plots
        self.axes.append(plt.subplot(223, projection='3d'))
        self.axes.append(plt.subplot(224, projection='3d'))
        # install GUI event handlers for interactive analysis
        self.figure.canvas.mpl_connect('key_press_event', self.keyPress)
        self.figure.canvas.mpl_connect('button_press_event', self.onClick)
        self.figure.canvas.mpl_connect('button_release_event', self.onRelease)
        # draw it all (costly), store pixel buffers
        self.figure.canvas.draw()
        self.figure.tight_layout()
        mng = plt.get_current_fig_manager()
        #mng.full_screen_toggle() # specific for tkAgg
        # show
        plt.show()

        
