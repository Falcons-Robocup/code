#!/usr/bin/env python
#
# FALCONS // Jan Feitsma, June 2018


from tracePlotBase import *


class tracePlotLatency(tracePlotBase):

    def __init__(self, settings):
        tracePlotBase.__init__(self, settings)
        # require single-robot data
        if len(settings.robots) != 1:
            raise Exception("require single-robot data")
        self.robot = self.robots[self.settings.robots[0]]

    def _plot(self):
        # TODO: choose better function name w.r.t. plot() ...
        # localization
        wm = self.robot.wm.get()
        #self.axes[0].plot(wm.t, wm.currentX, '-', c='k', label='wmX')
        self.axes[0].plot(wm.t, wm.currentPhi, ':', c='k', label='wmPhi')
        #vis = self.robot.vis.get()
        #self.axes[0].plot(vis.t, vis.x, '-.o', c='k')
        #self.axes[0].plot(vis.t, -vis.x, '-.o', c='k', label='visX') # both, to cope with symmetry
        # objects: at least one ball on a given position (see measure protocol)
        # TODO: also an obstacle?
        # TODO: filter out data far from expected positions, to keep plots clean (use object distance 'r'?)
        bm = self.robot.bm.get()
        t = bm.t
        az = bm.az
        camX = bm.camX
        x = bm.x
        if True:
            # construct a select mask: ball was 2 meters in front of robot facing forward (to filter out all the rest, which cause a messy plot)
            mask = bm.r > 1.0
            mask = mask * (bm.r < 3.0)
            mask = mask * (bm.az > 1.0)
            mask = mask * (bm.az < 2.0)
            t = t[mask]
            az = az[mask]
            camX = camX[mask]
            x = x[mask]
        self.axes[0].plot(t, az, '-.', c='b', label='ballAz')
        ballRelX = x - camX
        self.axes[0].plot(t, ballRelX, '-.', c='r', label='ballRelX')
        # legend
        self.axes[0].legend(loc=2)
        # TODO zero axis / grid
        # make pretty time labels
        # TODO: if timerange is small enough, go to millisecond resolution
        self.figure.autofmt_xdate()
        locator = matplotlib.dates.AutoDateLocator(minticks=3)
        formatter = matplotlib.dates.DateFormatter('%H:%M:%S.%f')
        self.axes[0].xaxis.set_major_locator(locator)
        self.axes[0].xaxis.set_major_formatter(formatter)

    def plot(self):
        # setup figure layout
        self.figure, ax = plt.subplots()
        self.axes = []
        self.axes.append(plt.subplot(111))
        self.figure.canvas.set_window_title('Harry Plotter v2.0: r%d latency' % self.settings.robots[0])
        # plot data
        self._plot()
        # draw it all (costly), store pixel buffers
        self.figure.canvas.draw()
        self.figure.tight_layout()
        mng = plt.get_current_fig_manager()
        #mng.full_screen_toggle() # specific for tkAgg
        # show
        plt.show()

        
