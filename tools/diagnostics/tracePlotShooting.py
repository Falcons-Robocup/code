#!/usr/bin/env python
#
# FALCONS // Jan Feitsma, November 2017


from tracePlotBase import *


class tracePlotShooting(tracePlotBase):

    def __init__(self, settings):
        tracePlotBase.__init__(self, settings)
        self.shootIndex = 0
        self.shootEventLookBack = 2.0
        self.shootEventLookForward = 0.7
        self.shootEventTimestamps = []
        self.deltaZoom = 0.3
        
    def eventTimeRange(self, t):
        return (t-self.shootEventLookBack, t+self.shootEventLookForward)

    def set_ylims(self):
        self.axes[0].set_ylim(24.0, 38.0)
        self.axes[1].set_ylim(-self.deltaZoom, self.deltaZoom)
        self.axes[2].set_ylim(-0.1, 2*pi+0.1)
    
    def zoomShootEvent(self, shootIndex):
        self.axes[0].set_xlim(*self.eventTimeRange(self.shootEventTimestamps[shootIndex]))
        self.set_ylims()
        self.figure.canvas.draw() # too costly
        return
        # TODO: redrawing the entire figure is way too costly. It can easily take seconds...
        # Either use blitting or just draw data per event on demand (refactor plotEvents)
        print "zoom", shootIndex
        print "draw"
        self.figure.canvas.restore_region(self.background[0])
        self.figure.canvas.restore_region(self.background[1])
        self.figure.canvas.restore_region(self.background[2])
        self.figure.canvas.blit(self.axes[0].bbox)
        self.figure.canvas.blit(self.axes[1].bbox)
        self.figure.canvas.blit(self.axes[2].bbox)
        print "set_xlim"
        print "done"

    def storeShootEvents(self):
        self.shootEventTimestamps = []
        for robotId in self.robots.keys():
            r = self.robots[robotId]
            for e in r.events.get():
                if e.type == 'kick':
                    self.shootEventTimestamps.append(e.t)
        if len(self.shootEventTimestamps) == 0:
            raise Exception("no shoot events!")
        self.shootEventTimestamps.sort()
        
    def keyPress(self, event):
        if event.key == 'n': # next
            self.shootIndex = min(1 + self.shootIndex, len(self.shootEventTimestamps) - 1)
        if event.key == 'p': # previous
            self.shootIndex = max(self.shootIndex - 1, 0)
        if event.key == 'f': # first
            self.shootIndex = 0
        if event.key == 'l': # last
            self.shootIndex = len(self.shootEventTimestamps) - 1
        self.zoomShootEvent(self.shootIndex)

    def plotMeta(self):
        # set axes limits and legends
        # plots and rectangles are on some dummy location, only to make a nice & complete legend
        addBackgroundBlocks(self.axes[0], [0], [1], 0, 1, "#ffcc66", "BHdisabled")
        addBackgroundBlocks(self.axes[0], [0], [1], 0, 1, "#ff9999", "lostBall")
        addBackgroundBlocks(self.axes[0], [0], [1], 0, 1, "#ccff66", "hasBall")
        self.axes[0].plot([0, 0], [1, 1], '-', c='r', label='left')
        self.axes[0].plot([0, 0], [1, 1], '--', c='r', label='right')
        self.axes[0].set_ylabel('ballHandler angle', color='r')
        self.axes[0].legend(loc=2)
        self.axes[2].plot([0, 0], [1, 1], '-', c='b', label='currentPhi')
        self.axes[2].plot([0, 0], [1, 1], '--', c='b', label='targetPhi')
        self.axes[2].set_ylabel('move angle (phi)', color='b')
        self.axes[2].legend(loc=1)
        addBackgroundBlocks(self.axes[1], [0], [1], 0, 1, "#ccffff", "rotate")
        addBackgroundBlocks(self.axes[1], [0], [1], 0, 1, "#ffff99", "settle")
        addBackgroundBlocks(self.axes[1], [0], [1], 0, 1, "#ccff66", "shoot")
        self.axes[1].plot([0, 0], [1, 1], c='k', label='deltaPhi')
        self.axes[1].plot([0, 0], [1, 1], '--', c='r', label='accuracy')
        self.axes[1].legend(loc=2)
        self.set_ylims()

    def plotValues(self, r, timeRange):
        # get all data
        wm = r.wm.get(timeRange)
        pp = r.pp.get(timeRange)
        bh = r.bh.get(timeRange)
        mp = r.mp.get(timeRange)
        # plot time periods where BH is disabled as orange background color
        addBackgroundBlocks(self.axes[0], bh.t, [not(v1 and v2) for v1, v2 in zip(bh.enabledL, bh.enabledR)], 0.0, 50, "#ffcc66", 'BHdisabled')
        # plot ball possession time periods
        addBackgroundBlocks(self.axes[0], bh.t, [not v for v in bh.hasBall], 0.0, 30, "#ff9999", 'lostBall')
        addBackgroundBlocks(self.axes[0], bh.t, bh.hasBall, 0.0, 30, "#ccff66", 'hasBall')
        # plot ballHandler angles in twin axis in top plot
        self.axes[0].plot(bh.t, bh.angleL, '-', c='r', label='left')
        self.axes[0].plot(bh.t, bh.angleR, '--', c='r', label='right')
        # plot current and target angle in top plot
        self.axes[2].plot(wm.t, wm.currentPhi, '-', c='b', label='currentPhi')
        self.axes[2].plot(pp.t, pp.targetPhi, '--', c='b', label='targetPhi')
        # bottom plot phase background color (ROTATE, SETTLE, SHOOT)
        addBackgroundBlocks(self.axes[1], mp.t, [v == "ROTATE" for v in mp.phase], -1.0, 2.0, "#ccffff", 'rotate')
        addBackgroundBlocks(self.axes[1], mp.t, [v == "SETTLE" for v in mp.phase], -1.0, 2.0, "#ffff99", 'settle')
        addBackgroundBlocks(self.axes[1], mp.t, [v == "SHOOT"  for v in mp.phase], -1.0, 2.0, "#ccff66", 'shoot')
        # plot delta in bottom plot
        accuracy = numpy.array(mp.threshold, dtype=numpy.float)
        self.axes[1].plot(mp.t, mp.deltaPhi, c='k', label='deltaPhi')
        self.axes[1].plot(mp.t, accuracy, '--', c='r', label='accuracy')
        self.axes[1].plot(mp.t, -accuracy, '--', c='r')
        # prettify: add dotted line at y=zero
        self.axes[1].plot(timeRange, [0, 0], '-.', c='k')
            
    def plotEvents(self):
        # plot events
        eventTypes = ['shoot', 'pass', 'bhOff', 'bhOn', 'kick', 'setHeight', 'lostBall']
        colors = defaultdict(lambda: 'b')
        ry = defaultdict(lambda: 0.7)
        ry['bhOff'] = -0.7
        ry['bhOn'] = -0.7
        ry['kick'] = -0.3
        ry['setHeight'] = -0.5
        align = defaultdict(lambda: 'left')
        align['bhOff'] = 'right'
        align['kick'] = 'left'
        align['setHeight'] = 'right'
        for robotId in self.robots.keys():
            r = self.robots[robotId]
            for e in r.events.get():
                eventType = e.type
                if eventType in eventTypes:
                    #bbox=dict(facecolor='none', edgecolor='blue', pad=10.0))
                    if eventType in ['shoot', 'pass']:
                        s = eventType
                        # legacy shoot->pass label hack because motionPlanning would report shoot i.s.o. pass
                        if len(e.details) and "9.00, " not in e.details:
                            s = "pass"
                        # make nice label
                        label = timeStr(e.t) + "/x%04d: " % (int(e.t) % 10000) + "r" + str(robotId) + " " + s
                        if len(e.details):
                            label += ' (' + e.details + ')'
                        # plot
                        timeRange = self.eventTimeRange(e.t)
                        self.plotValues(r, timeRange)
                        self.axes[1].text(e.t - 0.5*self.shootEventLookBack, self.deltaZoom*0.9, label, color='blue') 
                        # do not show as regular event plot, this is too confusing (the kick is the one we are interested in, not start of green SHOOT phase)
                        continue
                    # label the event
                    s = eventType
                    if len(e.details):
                        s += ' (' + e.details + ')'
                    x = e.t
                    yy = [self.deltaZoom*ry[eventType], 0]
                    self.axes[1].plot([x, x], yy, '-', c=colors[eventType])
                    self.axes[1].text(x, yy[0], s, color='blue', horizontalalignment=align[eventType]) 

    def plot(self):
        # identify the shoot events and store for browsing
        self.storeShootEvents()
        # setup figure layout
        self.figure, ax = plt.subplots()
        self.axes = []
        self.axes.append(plt.subplot(211))
        self.axes.append(plt.subplot(212, sharex=self.axes[0]))
        self.axes.append(self.axes[0].twinx()) # twin axis on top plot
        self.figure.canvas.set_window_title('Harry Plotter v2.0: shooting')
        #self.figure.subplots_adjust(hspace=0.05)
        self.plotMeta()
        self.plotEvents()
        # TODO make pretty time labels ??  
        #axes[0].autofmt_xdate()
        #self._f.canvas.mpl_connect('resize_event', self.onresize)
        #line_ani = anim.FuncAnimation(self._f, self.update_lines, interval=10, blit=False)
        # install key-press event handler so we can browse shoot events 
        self.figure.canvas.mpl_connect('key_press_event', self.keyPress)
        # draw it all (costly), store pixel buffers
        self.figure.canvas.draw()
        self.background = {}
        self.background[0] = self.figure.canvas.copy_from_bbox(self.axes[0].bbox)
        self.background[1] = self.figure.canvas.copy_from_bbox(self.axes[1].bbox)
        self.background[2] = self.figure.canvas.copy_from_bbox(self.axes[2].bbox)
        # zoom to first shoot event
        self.zoomShootEvent(0)
        # show
        plt.show()
        
