# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import numpy as np
import traceback
from rdl_model import Model
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt



class LatencyModelData():
    def __init__(self):
        self.series1 = {}
        self.series2 = {}
        self.offset = None

class LatencyModel(Model):
    """
    Base class to perform common modeling operations.
    """
    def __init__(self, *args, **kwargs):
        Model.__init__(self, *args, **kwargs) # load RDL etc

    def model(self, t1, v1, t2, v2):
        """
        Model latency between two time series in current dataframe.
        Data selection (as 'phases') should already have been performed beforehand.
        t1 and t2 are the labels of the timestamp columns; v1 and v2 the labels of value columns.
        """
        df = self.dataframe
        phases = sorted(list(set(df['phase'].values)))
        phases.remove(0)
        if len(phases) == 0:
            raise Exception("no useful data (phases) identified")
        self.result = []
        def makeSeries(dfp, t, v):
            s = {}
            s['t_label'] = t
            s['v_label'] = v
            values = dfp[v].values
            mask = ~np.isnan(values)
            s['t_values'] = dfp[t].values[mask]
            s['v_values'] = values[mask]
            # polyfit
            s['poly'] = np.polyfit(s['v_values'], s['t_values'], 1)
            return s
        for phase in phases:
            # filter dataframe for this phase
            phaseMask = df['phase'] == phase
            dfp = df[phaseMask]
            # store data and calculate polynomial fit
            r = LatencyModelData()
            try:
                r.series1 = makeSeries(dfp, t1, v1)
                r.series2 = makeSeries(dfp, t2, v2)
                # calculate latency
                x0 = r.series2['v_values'][0]
                r.offset = np.polyval(r.series1['poly'] - r.series2['poly'], x0)
                #print "phase %d: x0=%16.6f, offset=%.3fs" % (phase, x0, r.offset)
                #print(vars(r))
            except Exception as e:
                traceback.print_exc()
                print("WARNING: failed to model phase " + str(phase))
            self.result.append(r)

    def printSummary(self):
        idx = 0
        n = 0
        total = 0.0
        for r in self.result:
            idx += 1
            if r.offset != None:
                print("measurement {:d}/{:d}: offset = {:7.3f}s".format(idx, len(self.result), r.offset))
                total += r.offset
                n += 1
            else:
                print("measurement {:d}/{:d}: failed".format(idx, len(self.result)))
        print("average offset = {:.3f}s".format(total / n))

    def plot(self, all=False):
        plt.figure()
        def plotLine(x1, x2, fit):
            line = fit[0] * np.array([x1, x2]) + fit[1]
            plt.plot(line, [x1, x2], 'k:')
        if all:
            r = self.result[0]
            plt.plot(self.dataframe[r.series1['t_label']], self.dataframe[r.series1['v_label']], 'bx', label=r.series1['v_label'])
            plt.plot(self.dataframe[r.series2['t_label']], self.dataframe[r.series2['v_label']], 'r+', label=r.series2['v_label'])
        first = True
        for r in self.result:
            if r.offset == None:
                continue
            if not all:
                if first:
                    plt.plot(r.series1['t_values'], r.series1['v_values'], 'bx', label=r.series1['v_label'])
                    plt.plot(r.series2['t_values'], r.series2['v_values'], 'r+', label=r.series2['v_label'])
                    first = False
                else:
                    plt.plot(r.series1['t_values'], r.series1['v_values'], 'bx')
                    plt.plot(r.series2['t_values'], r.series2['v_values'], 'r+')
            x1 = r.series2['v_values'][0]
            x2 = r.series2['v_values'][-1]
            plotLine(x1, x2, r.series1['poly'])
            plotLine(x1, x2, r.series2['poly'])
            s = "offset=%.3fs" % (r.offset)
            plt.text(r.series1['t_values'][0], r.series1['v_values'][0], s)
        plt.legend()
        plt.show()

