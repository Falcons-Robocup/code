# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3 -u


import os
import argparse
import datetime
from copy import copy
import numpy as np
import falconspy
import analyze_lib as lib
from collections import OrderedDict, defaultdict
import pandas as pd
from rdl_model import Model
import rdl_adapters
from plot_browser import Plot, PlotBrowser

import falconspy
import rtdb2tools


def parse_arguments():
    descriptionTxt = """Analyze intercept (pass acceptance) performance, write a report to stdout.
"""
    exampleTxt = """Examples output:

analyzed 70 intercept attempts by robot 5
 id                   timestamp gotBall  duration noBall                               verdict   details
  1  2020-02-08,15:42:49.543537   False   13.9000   0.0%  failed: ball never came close enough      3.9m
  2  2020-02-08,15:43:05.876877    True    2.6333   0.0%                                    OK
  3  2020-02-08,15:43:10.543537   False    2.4000   0.0%  failed: ball never came close enough      1.4m
  4  2020-02-08,15:43:14.510204    True    2.6000   0.0%                                    OK
.....
 66  2020-02-08,15:54:27.643577   False    5.2333  43.0%  failed: ball never came close enough      1.2m
 67  2020-02-08,15:54:33.510215   False    2.4667   0.0%  failed: ball never came close enough      12.1m
 68  2020-02-08,15:54:36.676902   False    1.9667   0.0%  failed: ball never came close enough      10.9m
 69  2020-02-08,15:54:39.476890   False    0.5000   0.0%  failed: intercept duration too short      0.5s
 70  2020-02-08,15:54:40.543602   False   12.2667   0.0%  failed: ball never came close enough      10.1m

 37.1% (n= 26) failed: ball never came close enough
 24.3% (n= 17) failed: intercept duration too short
 21.4% (n= 15) failed: unknown reason
 15.7% (n= 11) OK
  1.4% (n=  1) failed: ball bounced off

"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-p', '--plot', help='show a plot', action='store_true')
    parser.add_argument('-d', '--dump', help='dump dataframe ASCII tables to /tmp', action='store_true')
    parser.add_argument('-a', '--all', help='show as much as possible', action='store_true')
    parser.add_argument('-f', '--frequency', help='resample frequency to use (typically best to slightly undersample heartbeat)', type=float)
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()


class InterceptAttempt():
    def __init__(self):
        self.label = None
        self.age = 0
        self.timestamp = 0
        self.timestampStr = None
        self.interceptDuration = 0.0
        self.targetRobot = 0
        self.passAttempt = False
        self.passDistance = None # only if passAttempt == True
        self.passingRobot = None # only if passAttempt == True
        self.passDuration = None # only if passAttempt == True
        self.passPower = None # only if passAttempt == True
        self.ballSpeed = None # using worldModel
        self.ballCloseEnough = False
        self.ballClosestDistance = 9.9
        self.noBallCount = 0 # number of ticks
        self.noBallPercentage = 0
        self.gotBall = False
        self.gotBallClean = False
        self.bhAcceptClean = [False, False] # left, right - 'clean' means no dip during ball acceptance
        self.bhAcceptSpike = [False, False] # left, right - 'spike' means a brief high reading around the time of acceptance
        self.ballAzimuth = 0.0 # at fixed distance close to robot
        self.verdict = "" # a readable string, like "OK", "ball never got close", ...
        self.verdictDetails = "" # custom details, not used in counting
        self.data = None # partial pandas dataframe
    def __repr__(self):
        v = copy(vars(self))
        del v['data'] # would heavily clutter output
        return str(v)
    def as_dict(self, all=False):
        v = copy(vars(self))
        if all:
            return v
        # return only the most interesting information, to be displayed in a concise table
        keys = ['label', 'timestampStr', 'gotBall', 'interceptDuration', 'noBallPercentage', 'verdict', 'verdictDetails']
        return OrderedDict([(k,v[k]) for k in keys])


class InterceptSettings():
    def __init__(self):
        self.closeByDistance = 1.0
        self.speedAveragingDuration = 0.5
        self.ballPossessionTimeout = 1.0 # timer starts when ball comes within 'closeByDistance'
        self.interceptMinimumDuration = 1.0
        self.extensionRowCount = 3
        self.extensionDuration = 1.0 # seconds after ball has gotten close


class InterceptAnalyzer(Model):

    def __init__(self, *args, **kwargs):
        Model.__init__(self, *args, **kwargs) # load RDL etc
        self.settings = InterceptSettings()
        self.info("analyzing intercept attempts ...")
        self.makeDataFrame()
        self.analyzeInterceptAttempts()
        self.info(" done\n")

    def attemptsAsDataframe(self):
        columns = self.interceptAttempts[0].as_dict().keys()
        return pd.DataFrame([a.as_dict() for a in self.interceptAttempts], columns=columns)

    def printAll(self):
        """
        Print all analyzed intercepts with details.
        """
        for it in range(len(self.interceptAttempts)):
            attempt = self.interceptAttempts[it]
            if attempt.interceptDuration > 1:
                print("")
                print("intercept attempt {:d}/{:d}:".format(1+it, len(self.interceptAttempts)))
                print(str(attempt))

    def printSummary(self):
        """
        Print a summary of succes- and failure rates of all intercepts.
        """
        total = len(self.interceptAttempts)
        print("")
        print("analyzed {:d} intercept attempts by robot {:d}".format(total, self.agent))
        # summary table with one intercept attempt per row
        df = analyzer.attemptsAsDataframe()
        df = df.rename(columns={"label": "id", "timestampStr": "timestamp", "interceptDuration": "duration", "verdictDetails": "details", "noBallPercentage": "noBall"})
        print(df.to_string(index=False))
        # count the type of verdicts and display in decreasing occurrence
        verdictCounts = defaultdict(lambda: 0)
        for it in range(len(self.interceptAttempts)):
            attempt = self.interceptAttempts[it]
            verdictCounts[attempt.verdict] += 1
        print("")
        for k, v in sorted(verdictCounts.items(), key=lambda item: -item[1]):
            print("{:5.1f}% (n={:3d}) {:s}".format(100.0 * v / total, v, k))
        print("")

    def plot(self):
        """
        Make a plot of the most interesting data columns and browse through the data.
        """
        b = PlotBrowser(title='intercept browser')
        b.defaultXlim = [-2.5, 1.0] # timestamps relative to t=0, which is typically the ball coming close, or end-of-dataframe
        b.defaultYlim = [-3.0, 3.0]
        b.defaultLegendLoc = 'lower left' # most interesting data is positive, like bh readings and ball distance
        # copy the interesting data to Plot objects
        for it in range(len(self.interceptAttempts)):
            attempt = self.interceptAttempts[it]
            t = attempt.data['plot_time']
            # store the curves to be plotted
            p = Plot()
            p.addCurve(t, attempt.data['ball_azimuth'], 'ball azimuth (vis/wm)', 'm+-')
            p.addCurve(t, attempt.data['ball_distance'], 'ball distance (wm)', 'b+-')
            p.addCurve(t, attempt.data['ball_x_rcs'], 'ball x RCS (wm)', 'r+:')
            p.addCurve(t, attempt.data['ball_vx_rcs'], 'ball vx RCS (wm)', 'c+:')
            p.addCurve(t, attempt.data['hasball'], 'ball possession (wm)', 'g+-')
            p.addCurve(t, attempt.data['velsp_x'], 'vx setpoint (pp)', 'r+-.')
            p.addCurve(t, attempt.data['vel_x_rcs'], 'vx feedback (wm)', 'r+-')
            p.addCurve(t, attempt.data['velsp_Rz'], 'vrz setpoint (pp)', 'k+-.')
            p.addCurve(t, attempt.data['vel_Rz'], 'vrz feedback (wm)', 'k+-')
            p.addCurve(t, attempt.data['bh_left'], 'ballhandler left (bh)', '+-', c='darkgoldenrod')
            p.addCurve(t, attempt.data['bh_right'], 'ballhandler right (bh)', '+-', c='darkorange')
            # TODO: mp/pp target RCS?
            # label: timestamp + verdict
            mainLabelStr = "r{} intercept {}/{} at {} (age={:.2f}): {}".format(self.agent, attempt.label, len(self.interceptAttempts), attempt.timestampStr[11:23], attempt.age, attempt.verdict)
            if attempt.verdictDetails:
                mainLabelStr += " (" + attempt.verdictDetails + ")"
            backgroundcolor = 'orange'
            if attempt.verdict == "OK":
                backgroundcolor = 'lightgreen'
            p.addLabel(-2.0, 2.5, mainLabelStr, backgroundcolor=backgroundcolor, fontsize=18)
            b.plots.append(p)
        # show the figure
        b.select(0)
        b.show()

    def makeDataFrame(self):
        """
        Create pandas dataframe for analyzing intercept performance.
        """
        # construct the pandas DataFrame object
        Model.makeDataFrame(self,
            rdl_adapters.WorldModel(),
            rdl_adapters.VisionBestBall(), # raw vision data contains far too many false positives, so use wm selection
            rdl_adapters.Actuation(),
            rdl_adapters.BallHandling())
        self.extendDataFrame()

    def extendDataFrameRcs(self, df):
        """
        Add some columns, converting FCS to RCS coordinates.
        """
        angle = np.pi*0.5 - df.get('pos_Rz').values
        s = np.sin(angle)
        c = np.cos(angle)
        # robot velocity in RCS
        vx = df.get('vel_x').values
        vy = df.get('vel_y').values
        df.insert(len(df.columns), 'vel_x_rcs', c * vx - s * vy)
        df.insert(len(df.columns), 'vel_y_rcs', s * vx + c * vy)
        # (wm) ball position in RCS
        bx = df.get('ball_x').values - df.get('pos_x').values
        by = df.get('ball_y').values - df.get('pos_y').values
        df.insert(len(df.columns), 'ball_x_rcs', c * bx - s * by)
        df.insert(len(df.columns), 'ball_y_rcs', s * bx + c * by)
        # (wm) ball velocity in RCS
        vx = df.get('ball_vx').values
        vy = df.get('ball_vy').values
        df.insert(len(df.columns), 'ball_vx_rcs', c * vx - s * vy)
        df.insert(len(df.columns), 'ball_vy_rcs', s * vx + c * vy)

    def extendDataFrame(self):
        """
        Extend pandas dataframe with extra columns, apply necessary filtering / smoothening operations for analysis.
        """
        df = self.dataframe
        # convert some columns from FCS to RCS
        self.extendDataFrameRcs(df)
        # ball speed according to worldModel (may not be very reliable, use with caution)
        df['ball_speed'] = np.sqrt(np.square(df['ball_vx']) + np.square(df['ball_vy']))
        # when was robot attempting to intercept
        df['intercepting'] = (df['action'] == 10) # TODO also allow getBall? functionally similar..
        # analyze ball distance and if it was monotonously decreasing (need a bit of smoothening)
        # ball 'radius' is actually distance to camera -> calculate distance on floor
        CAMHEIGHT = 0.75
        df['ball_distance_vision'] = np.sqrt(np.square(df['ball_radius']) - CAMHEIGHT**2)
        df['ball_distance_wm'] = np.sqrt(np.square(df['ball_y_rcs']) + np.square(df['ball_x_rcs']))
        df['ball_distance'] = df['ball_distance_wm'] # vision produces too many fake balls currently
        df = lib.smoothen(df, ['ball_distance'])
        df['ball_coming_closer'] = (df['ball_distance'] - np.roll(df['ball_distance'], 1)) <= 0
        df['plot_time'] = df['wm_t']
        # tag the intercept phases
        lib.make_phases(df, lambda row: row['intercepting'] == True)
        # iterate over phases
        def calcTimeWrtCloseBall(df):
            phases = sorted(list(set(df['phase'].values)))
            phases.remove(0)
            for phase in phases:
                # filter dataframe for this phase
                phaseMask = df['phase'] == phase
                dfp = df[phaseMask]
                # calculate timestamp where ball comes close to robot
                closeMask = dfp['ball_distance'] < self.settings.closeByDistance
                if len(dfp[closeMask]):
                    t = dfp[closeMask]['age'].values[0]
                    df.loc[phaseMask, 'time_wrt_closeball'] = dfp['age'] - t
                    df.loc[phaseMask, 'plot_time'] = dfp['age'] - t # default: plot using time_wrt_closeball
                else:
                    t = df[phaseMask]['wm_t'].values[-1]
                    df.loc[phaseMask, 'plot_time'] = dfp['wm_t'] - t # fallback: plot w.r.t time where 0 is end of dataframe
        calcTimeWrtCloseBall(df)
        # sanity check: do we have enough data
        if not 'time_wrt_closeball' in df.columns:
            self.dumpdf(df, filename='/tmp/dataframe.txt')
            raise Exception("cannot make sense of data") # can happen for instance if sampling rate is too high
        # extend each phase a bit so we can analyze ball acceptance even after robot decides to stop intercepting
        extendCounter = 0
        for irow in range(1, len(df)):
            previousRow = df.loc[irow-1]
            thisRow = df.loc[irow]
            doExtend = False
            # extension based on time after ball coming close
            if thisRow['phase'] == 0:
                if previousRow['time_wrt_closeball'] < self.settings.extensionDuration:
                    doExtend = True
            # extension based on row count
            if thisRow['phase'] == 0:
                if extendCounter < self.settings.extensionRowCount:
                    doExtend = True
                    extendCounter += 1
            else:
                extendCounter = 0
            if doExtend:
                df.loc[irow, 'phase'] = previousRow['phase']
                df.loc[irow, 'time_wrt_closeball'] = previousRow['time_wrt_closeball'] + thisRow['age'] - previousRow['age']
        # re-calculate phases because some might have merged
        lib.make_phases(df, lambda row: row['phase'] > 0)
        calcTimeWrtCloseBall(df)
        self.dataframe = df

    def analyzeInterceptAttempts(self):
        self.interceptAttempts = []
        df = self.dataframe
        # iterate over phases
        phases = sorted(list(set(df['phase'].values)))
        if 0 in phases:
            phases.remove(0)
        for phase in phases:
            # filter dataframe for this phase
            dfp = df[df['phase'] == phase]
            attempt = self.analyzeInterceptAttempt(dfp, phase)
            self.interceptAttempts.append(attempt)

    def analyzeInterceptAttempt(self, dfp, label):
        attempt = InterceptAttempt()
        attempt.label = label
        attempt.data = dfp
        # fill in details
        attempt.age = dfp.iloc[0]['age']
        attempt.timestamp = self.rdl.header.creation + attempt.age
        attempt.timestampStr = datetime.datetime.fromtimestamp(float(attempt.timestamp)).strftime("%Y-%m-%d,%H:%M:%S.%f")
        attempt.interceptDuration = dfp.iloc[-1]['age'] - dfp.iloc[0]['age']
        attempt.targetRobot = self.agent
        # also use self.events
        #for e in self.events:
        #    if timestamp in __range:
        #        TODO passAttempt, passDistance, passingRobot, passDuration, passPower
        attempt.ballClosestDistance = dfp['ball_distance'].min()
        # ball speed: average over a period where the ball speed is most reliable
        mask = dfp['time_wrt_closeball'] > -self.settings.speedAveragingDuration
        mask &= dfp['time_wrt_closeball'] < 0.0
        attempt.ballSpeed = np.mean(dfp[mask]['ball_speed'])
        attempt.ballCloseEnough = attempt.ballClosestDistance < self.settings.closeByDistance
        # ball angle closeby, should highly correlate with the first ballHandler being touched by the ball
        closeMask = dfp['ball_distance'] < self.settings.closeByDistance
        if len(dfp[closeMask]):
            self.ballAzimuth = dfp[closeMask]['ball_azimuth'].values[0] # TODO: look even closer?
        attempt.noBallCount = dfp['ball_x'].isnull().sum()
        attempt.noBallPercentage = "%.1f%%" % (100.0 * attempt.noBallCount / len(dfp))
        # inspect the second after ball comes close enough (settings.ballPossessionTimeout = 1.0)
        # during that second has elapsed, robot should have gained ball possession
        # and if not, during this second we look for ball handler spikes
        dfa = dfp[(dfp['time_wrt_closeball'] >= 0.0) & (dfp['time_wrt_closeball'] <= self.settings.ballPossessionTimeout)]
        attempt.gotBall = any(dfa['hasball'].values == True)
        # analyze ballhandler reading, detect spikes
        sides = ['left', 'right']
        for side in [0,1]:
            v = dfa['bh_'+sides[side]].values
            idxHigh = np.where(v > 0.3)[0]
            idxLow = np.where(v < 0.3)[0]
            spike = False
            if len(idxLow) and len(idxHigh):
                spike = max(idxLow) > min(idxHigh)
            attempt.bhAcceptSpike[side] = spike
            # for now, there is no special logic to determine 'clean' acceptance other than checking for spike
            attempt.bhAcceptClean[side] = not spike
        attempt.gotBallClean = attempt.gotBall and attempt.bhAcceptClean[0] == True and attempt.bhAcceptClean[1] == True
        # make a verdict
        if attempt.interceptDuration < 1.0:
            attempt.verdict = "failed: intercept duration too short"
            attempt.verdictDetails = "{:.1f}s".format(attempt.interceptDuration)
        else:
            if not attempt.ballCloseEnough:
                attempt.verdict = "failed: ball never came close enough"
                attempt.verdictDetails = "{:.1f}m".format(attempt.ballClosestDistance)
            else:
                if attempt.gotBallClean:
                    attempt.verdict = "OK"
                    attempt.verdictDetails = ""
                else:
                    if attempt.gotBall:
                        attempt.verdict = "OK-ish"
                        badBh = [["left", "right"][bh] for bh in range(2) if not attempt.bhAcceptClean[bh]]
                        if len(badBh) == 2:
                            attempt.verdictDetails = "both ballHandlers dipped"
                        elif len(badBh) == 1:
                            attempt.verdictDetails = badBh[0] + " ballHandler dipped"
                        else:
                            raise Exception("something wrong in logic?") # should not happen unless definition of OK-ish was extended
                    else:
                        # robot did not get the ball (in time)
                        # check for spikes in bh readings, to see from which ballHandler it bounced
                        spikeBh = [["left", "right"][bh] for bh in range(2) if attempt.bhAcceptSpike[bh]]
                        if len(spikeBh) == 2:
                            attempt.verdict = "failed: ball bounced off"
                            attempt.verdictDetails = "both ballHandlers spiked"
                        elif len(spikeBh) == 1:
                            attempt.verdict = "failed: ball bounced off"
                            attempt.verdictDetails = spikeBh[0] + " ballHandler spiked"
                        else:
                            attempt.verdict = "failed: unknown reason"
                            attempt.verdictDetails = "" # ball came close but no ballHandlers spiked, was the robot facing completely wrong perhaps?
        return attempt


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name, choose suitable data sampling frequency
    agent = args.robot
    frequency = args.frequency
    if agent == 0:
        agent = lib.get_agent_from_rdl_filename(args.rdlfile)
        if frequency == None:
            frequency = 25.0 # subsample robot 30Hz heartbeat
    else:
        if frequency == None:
            frequency = 15.0
    # run
    analyzer = InterceptAnalyzer(args.rdlfile, agent, resampleFrequency=frequency)
    # write details to file?
    if args.dump:
        # first clear any existing dataframe dumps
        os.system('rm /tmp/dataframe*.txt 2>/dev/null')
        for attempt in analyzer.interceptAttempts:
            analyzer.dumpdf(attempt.data, filename='/tmp/dataframe'+str(attempt.label)+'.txt')
        analyzer.dumpdf(analyzer.dataframe, filename='/tmp/dataframe.txt')
    # display results
    if args.all:
        analyzer.printAll()
    # display summary table
    analyzer.printSummary()
    # plot?
    if args.plot:
        analyzer.plot()

