# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import time, datetime
import traceback
from collections import OrderedDict
import numpy as np
import pandas as pd
pd.set_option('mode.chained_assignment','raise')
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from matplotlib.patches import Ellipse, Rectangle, Circle
from plot_browser import PlotBrowser, Plot
from plot_field import drawField


NUMROBOTS=7


# TODO
# * factor out drawField for reuse in or instance POI plotter
# * rotate axis to be consistent with FCS and remove dirty workaroundtricks (hard!)
# * mouse-over detailed data inspection?
# * refactoring


class FieldPlot(Plot):
    def __init__(self):
        Plot.__init__(self)
    def plot(self, ax, legendLoc=None):
        # draw field before overlaying data using regular plot()
        ax.clear()
        ax.axis('equal')
        drawField(ax)
        Plot.plot(self, ax, clear=False, legendLoc=legendLoc)


class BallPlotter():
    """
    Construct Plot objects based on balldata.
    """
    def __init__(self, show=True):
        self.robotColors = ['red', 'blue', 'green', 'gold', 'gray', 'orange', 'brown'] * 3
        self.fieldPlot = None
        self.timelinePlot = None
        # labels for legend
        self.fieldLabels = OrderedDict()
        self.fieldLabels['posr'] = 'robot position r'
        self.fieldLabels['lbmr'] = 'ball measurement r'
        self.fieldLabels['lbrr'] = 'ball result r'
        self.fieldLabels['hbmr'] = None # small layouting delta only, no legend
        self.fieldLabels['hbrr'] = None
        self.timelineLabels = OrderedDict()
        for dof in "xyz":
            self.timelineLabels['lbmr'+dof] = 'ball measurement ' + dof + ' r'
            #self.timelineLabels['lbrr'+dof] = 'ball result ' + dof + ' r' # not useful anymore
        #for dof in "aer":
        #    self.timelineLabels['lbmr'+dof] = 'ball measurement ' + dof + ' r'
        # layouting
        self.timelineLookback = 1.0
        self.timelineLegendPadding = 1.0
        # overlays
        self.closestTrackerLabel = None

    def setupEventCallback(self, fig):
        #fig.canvas.mpl_connect('button_press_event', self.mouseEventCallback)
        fig.canvas.mpl_connect('motion_notify_event', self.mouseEventCallback)

    def mouseEventCallback(self, event, *args):
        if event.inaxes:
            # replot
            self.fieldPlot.plot(event.inaxes)
            self.highlightClosest(event, self.dataframeResults, 'tracker')
            self.highlightClosest(event, self.dataframeMeasurements, 'measurement')
            event.inaxes.figure.canvas.draw()

    def highlightClosest(self, event, df, what):
        # find tracker id closest to this coordinate
        # un-rotate axes coordinates to get field FCS
        xf = -event.ydata
        yf = event.xdata
        ax = event.inaxes
        print("click at (x,y) = ({.2f},{.2f})".format(xf, yf))
        d2 = np.sqrt((xf - df['x'].values)**2 + (yf - df['y'].values)**2)
        minDist = d2[np.argmin(d2)]
        if minDist < 1.0: # otherwise don't print
            # dump all details to stdout
            v = df.loc[np.argmin(d2)]
            # highlight the tracker by placing a label on current position
            details = what + ': ' + str(v.trackerId) + '\nrobot: ' + str(v.robotId)
            ax.text(event.xdata, event.ydata, details)

    def refreshPlots(self):
        if self.fieldPlot == None:
            self.fieldPlot = FieldPlot() # autozoom
        if self.timelinePlot == None:
            self.timelinePlot = Plot()

    def plot(self, balldata):
        self.balldata = balldata
        # construct/clear figures if needed
        self.refreshPlots()
        # calculate plottables
        plottables = self.calculate(balldata)
        # construct field plots
        markers = {'posr': 'x', 'lbmr': '^', 'hbmr': '^', 'lbrr': 'o', 'hbrr': 'o'}
        markersize = 12
        for p in plottables:
            (shortName, robotId, dfr) = p
            col = self.robotColors[robotId]
            if self.fieldLabels.has_key(shortName):
                longName = None
                if self.fieldLabels[shortName]:
                    longName = self.fieldLabels[shortName] + str(robotId)
                # invert X and Y for landscape-mode layout
                self.fieldPlot.addCurve(dfr['y'].values, -dfr['x'].values, longName, markers[shortName], c=col, markersize=markersize)
        # construct timeline plots
        dofs = 'xyzaer'
        markers = {'lbmr': '^v<x+*', 'lbrr': 'osD'}
        for p in plottables:
            (shortName, robotId, dfr) = p
            col = self.robotColors[robotId]
            for idof in range(len(dofs)):
                dof = dofs[idof]
                shortName = p[0] + dof
                if self.timelineLabels.has_key(shortName):
                    longName = self.timelineLabels[shortName] + str(robotId)
                    values = dfr[dof].values
                    self.timelinePlot.addCurve(dfr['t'].values, values, longName, c=col, linestyle='none', markersize=markersize, marker=markers[shortName[0:4]][idof])
        # titles
        title = 'timestamp: '
        if len(balldata.ballDiagnostics.items()) == 1:
            t = balldata.ballDiagnostics.items()[0][1].timestamp
            ts = datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")
            title += ts
        self.fieldPlot.addLabel(-8.0, 6.5, title, backgroundcolor='lightgrey', fontsize=18)
        self.timelinePlot.addLabel(-1.0, 4.0, title, backgroundcolor='lightgrey', fontsize=18)
        # connect plot callbacks
        self.timelinePlot.callback = self.timelineAxisLayouter
        self.fieldPlot.callback = self.fieldAxisLayouter
        # store data(frames) for use in callbacks
        self.plottables = plottables

    def fieldAxisLayouter(self, ax):
        # TODO get dimensions from environmentField
        ax.set_ylim(-7, 7)
        ax.set_xlim(-10, 17) # extra space to the right for legend
        ax.legend(loc='lower right')

    def timelineAxisLayouter(self, ax):
        ax.set_ylim(-5, 5)
        ax.set_xlim(-self.timelineLookback-0.2, 0.2+self.timelineLegendPadding)
        ax.legend(loc='lower right')

    def calculate(self, balldata):
        """
        Calculate data to be plotted, without layouting and without splitting field/timeline plotting.
        """
        plottables = [] 
        # settings
        highElevation = 0.0
        highBallZ = 0.7
        # helper
        def addPlottable(name, robotId, dfr):
            if len(dfr):
                plottables.append((name, robotId, dfr))
        # robot vision ball measurements
        #print "robot vision ball measurements"
        df = balldata.getBallMeasurements()
        # TODO check if df is not empty
        print(df.to_string())
        df['x'] = df['ballFcsX']
        df['y'] = df['ballFcsY']
        df['z'] = df['ballFcsZ']
        self.dataframeMeasurements = df
        print(df.to_string())
        if len(df) > 0:
            df_empty = df.iloc[0:0]
            for robotId in range(0, NUMROBOTS+1):
                # get applicable data
                dfr = df[df['robotId'] == robotId].copy()
                if len(dfr) == 0:
                    continue
                #print "robotId", robotId
                #print dfr.to_string()
                # use FCS calculated ball positions
                dfr['x'] = dfr['ballFcsX']
                dfr['y'] = dfr['ballFcsY']
                dfr['z'] = dfr['ballFcsZ']
                # abbreviate (azimuth, elevation, radius) to (a, e, r), for easy plotting in timeline
                dfr['a'] = dfr['azimuth']
                dfr['e'] = dfr['elevation']
                dfr['r'] = dfr['radius']
                # TODO: timestamp t and delta positions w.r.t tracker result
                # split low from high balls
                dfr_low = dfr[dfr['elevation'] < highElevation]
                dfr_high = dfr[dfr['elevation'] >= highElevation]
                addPlottable('lbmr', robotId, dfr_low)
                addPlottable('hbmr', robotId, dfr_high)
                # use robot positions (FCS) from camera/ray origins
                dfr['x'] = dfr['cameraX']
                dfr['y'] = dfr['cameraY']
                addPlottable('posr', robotId, dfr)
        # robot worldModel ball tracking
        #print "robot worldModel ball tracking"
        df = balldata.getBallResults()
        self.dataframeResults = df
        #print df.to_string()
        if len(df) > 0:
            df_empty = df.iloc[0:0]
            for robotId in range(0, NUMROBOTS+1):
                # get applicable data
                dfr = df[df['robotId'] == robotId].copy()
                if len(dfr) == 0:
                    continue
                #print "robotId", robotId
                #print dfr.to_string()
                # split low from high balls
                dfr_low = dfr[dfr['z'] < highBallZ]
                dfr_high = dfr[dfr['z'] >= highBallZ]
                addPlottable('lbrr', robotId, dfr_low)
                addPlottable('hbrr', robotId, dfr_high)
        # TODO associated and outlier-tagged measurement (via tracker diagnostics)
        #print "result plottables", len(plottables)
        return plottables

        # TODO cleanup below
        
    def initAnimationField(self):
        """
        Prepare animation of the field plot.
        Called at init time, but also on resize events and when framecounter reaches the limit.
        """
        p = self.fieldPlot
        if not p.first:
            return []
        self.drawField(p.ax)
        p.first = False
        # iterate over selected agents
        p.plotdata = {}
        m = 100
        for robotId in range(0, NUMROBOTS+1):
            col = self.robotColors[robotId]
            # abbreviate the plot element key names, for example lbmr = lowBallMeasurementsRobot
            p.plotdata['posr'+str(robotId)] = p.ax.scatter([], [], m, c=col, marker='x')
            p.plotdata['lbmr'+str(robotId)] = p.ax.scatter([], [], m, c=col, marker='^')
            p.plotdata['hbmr'+str(robotId)] = p.ax.scatter([], [], m*0.5, c=col, marker='^', alpha=0.5)
            p.plotdata['lbrr'+str(robotId)] = p.ax.scatter([], [], m, c=col, marker='o')
            p.plotdata['hbrr'+str(robotId)] = p.ax.scatter([], [], m*0.5, c=col, marker='o', alpha=0.5)
        return p.plotdata.values()

    def initAnimationTimeline(self):
        """
        Prepare animation of the timeline plot.
        """
        p = self.timelinePlot
        if not p.first:
            return []
        p.first = False
        p.ax.grid(True)
        p.plotdata = {}
        m = 100
        dofs = 'xyzaer'
        markers_bm = '^v<x+*'
        markers_br = 'osD'
        for robotId in range(0, NUMROBOTS+1):
            col = self.robotColors[robotId]
            for idof in range(len(dofs)):
                dof = dofs[idof]
                # abbreviate the plot element key names, for example lbmr = lowBallMeasurementsRobot
                if idof < len(markers_bm):
                    marker = markers_bm[idof]
                    p.plotdata['lbmr'+dof+str(robotId)] = p.ax.scatter([], [], m, c=col, marker=marker)
                    p.plotdata['hbmr'+dof+str(robotId)] = p.ax.scatter([], [], m*0.5, c=col, marker=marker, alpha=0.5)
                if idof < len(markers_br):
                    marker = markers_br[idof]
                    p.plotdata['lbrr'+dof+str(robotId)] = p.ax.scatter([], [], m, c=col, marker=marker)
                    p.plotdata['hbrr'+dof+str(robotId)] = p.ax.scatter([], [], m*0.5, c=col, marker=marker, alpha=0.5)
        return p.plotdata.values()

    def updateFieldLegend(self):
        # update labels to include counts; hide the ones which have no data
        p = self.fieldPlot
        S = []
        L = []
        for robotId in range(0, NUMROBOTS+1):
            for key in self.fieldLabels.keys():
                s = p.plotdata[key+str(robotId)]
                n = len(s.get_offsets())
                lbl = ''
                if n > 0:
                    lbl = self.fieldLabels[key] + str(robotId) + ' (' + str(n) + ')'
                s.set_label(lbl)
        return p.ax.legend()

    def updateTimelineLegend(self):
        p = self.timelinePlot
        S = []
        L = []
        for robotId in range(0, NUMROBOTS+1):
            for key in self.timelineLabels.keys():
                s = p.plotdata[key+str(robotId)]
                n = len(s.get_offsets())
                lbl = ''
                if n > 0:
                    lbl = self.timelineLabels[key] + str(robotId)
                s.set_label(lbl)
        return p.ax.legend()


    def animateField(self, arg):
        """
        (Re-)draw the field figure by modifying data to plot.
        This function is to be called continuously at some frequency by matplotlib.FuncAnimation.
        """
        if self.shutdown:
            return []
        self.calculate()
        updated = []
        def _set_offsets(key, robotId, x, y):
            # wrapper to set scatter data via its function set_offsets
            # make Nx2 array and invert x,y for 'landscape' mode visualization
            v = np.concatenate((np.mat(y).T, -np.mat(x).T), axis=1) # invert x axis for now - TODO: rotate axis
            s = self.fieldPlot.plotdata[key+str(robotId)]
            s.set_offsets(v)
            updated.append(s)
        # modify plot for each plottable
        for (key, robotId, df) in self.plottables:
            if df.empty:
                _set_offsets(key, robotId, [], [])
            else:
                _set_offsets(key, robotId, df['x'], df['y'])
        # dynamic legend
        leg = self.updateFieldLegend()
        if leg != None:
            updated.append(leg)
        return updated

    def animateTimeline(self, arg):
        if self.shutdown:
            return []
        updated = []
        def _set_data(key, robotId, t, v):
            vv = np.concatenate((np.mat(t).T, np.mat(v).T), axis=1)
            s = self.timelinePlot.plotdata[key+str(robotId)]
            s.set_offsets(vv)
            updated.append(s)
        # correct timeline with 'current' time
        t0 = 0
        if self.plotControl.ageSelect != None:
            t0 = float(self.t0 + self.plotControl.ageSelect)
        # modify plot for each plottable
        for (key, robotId, df) in self.plottables:
            if "posr" in key:
                continue # ignore
            # plot several columns
            for dof in "xyzaer":
                if not self.timelineLabels.has_key(key+dof):
                    continue
                if df.empty:
                    _set_data(key+dof, robotId, [], [])
                else:
                    _set_data(key+dof, robotId, df['timestamp'].values - t0, df[dof].values)
        # axis scaling - TODO autoscale? TODO allow zooming
        self.timelinePlot.ax.relim()
        self.timelinePlot.ax.set_ylim(-5, 5)
        self.timelinePlot.ax.set_xlim(-self.plotControl.dataTimeout-0.2, 0.2) # tweak because data might be slightly offsetted into the future
        # dynamic legend
        leg = self.updateTimelineLegend()
        if leg != None:
            updated.append(leg)
        return updated

    def run(self):
        # wrapper to prevent program hanging in case of an exception
        try:
            self._run()
        except Exception as e:
            traceback.print_exc()

    def _run(self):
        # create timeline figure
        self.timelinePlot = PlotWindow('timeline plot', maximize=True)
        # create field plot figure
        self.fieldPlot = PlotWindow('field plot', maximize=True)
        # make sure shutdown triggers when any window is closed
        # TODO catch signals and exceptions properly
        def onClose(*args):
            do_shutdown = not self.shutdown
            self.shutdown = True
            if do_shutdown:
                plt.close('all')
        self.fieldPlot.fig.canvas.mpl_connect('close_event', onClose)
        self.timelinePlot.fig.canvas.mpl_connect('close_event', onClose)
        # show and run the mainloop
        plt.show() # blocks

