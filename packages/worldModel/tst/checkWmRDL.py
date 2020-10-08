""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

import sys, os
import argparse
import unittest
unittest.TestLoader.sortTestMethodsUsing = None
import yaml
import numpy as np
import time, datetime

# Falcons libraries
import falconspy
from rdlLib import RDLFile # from package logging
from WorldModelAnalyzer import WorldModelAnalyzer

# use utilities and data massaging classes from package diagnostics
from rdl_model import Model
import rdl_adapters
import analyze_robotstate


# TODO: continue on assert (like gtest 'expect')
# TODO: refactoring?



def parse_arguments():
    parser = argparse.ArgumentParser(description="Apply checks from an annotation file to given RDL.")
    parser.add_argument('annotation', help='annotation yaml file to run', type=str)
    parser.add_argument('-t', '--tolerances', help='tolerance specification yaml file', type=str, default='tolerances.yaml')
    parser.add_argument('--rdlfile', help='optional RDL filename overrule (default take the referred RDL from annotation file)', type=str)
    parser.add_argument('-r', '--robot', help='robot perspective to take, default guess from RDL filename', type=int)
    parser.add_argument('-d', '--dumpdataframe', help='dump the dataframe used in some checks to given file', type=str)
    parser.add_argument('-v', '--verbose', help='display extra details and statistics', action='store_true')
    return parser.parse_args()


#####################################################
# Base testcase class (with some utility functions) #
#####################################################

# naming convention: type X will lead to an instantiation of XAnnotationTestCase
# calling convention: each testcase first gets common argument 'dataHandle', followed by testcase-specific arguments consistenly named in the yaml


class _BaseAnnotationTestCase(unittest.TestCase):
    _counter = 0
    def __init__(self, dataHandle, verbose, **kwargs):
        unittest.TestCase.__init__(self)
        _BaseAnnotationTestCase._counter += 1
        self._id = _BaseAnnotationTestCase._counter
        self.verbose = verbose
        self.kwargs = kwargs
        self.dataHandle = dataHandle

    # since tests can be instantiated multiple times, display instance counter as well as parameters
    def __str__(self):
        return "%s/%d (%s)" % (type(self).__name__, self._id, str(self.kwargs))
    def id(self):
        # compatibility with rosunit xmlrunner, which prints as test name whatever comes after '.':
        #    (self._class, self._method) = test.id().rsplit(".", 1)
        return ".%s/%d" % (type(self).__name__.replace('.runTest', ''), self._id)

    def convertAgeToTimeString(self, age):
        return (self.dataHandle.rdl.header.creation + age).pretty()

    def convertYamlTimestampToAge(self, t):
        # in yaml, we use day-bound timestamp strings
        # standard yaml parser converts timestamps to float
        # example: '19:25:22.3' -> 69922.3
        # so 't' should be interpreted as seconds since midnight
        # in this function we need to compare with rdl creation
        # first truncate H:M:S from rdl creation timestamp
        tc = datetime.datetime.fromtimestamp(float(self.dataHandle.rdl.header.creation))
        dt0 = datetime.datetime(tc.year, tc.month, tc.day) # ignore the rest
        t0 = time.mktime(dt0.timetuple())
        # correct age since midnight
        return t + t0 - float(self.dataHandle.rdl.header.creation)


class RdlAnnotationDataFrame(Model):
    def __init__(self, *args, **kwargs):
        kwargs['verbose'] = False
        Model.__init__(self, *args, **kwargs) # load RDL etc
        # construct the pandas DataFrame object
        Model.makeDataFrame(self,
            rdl_adapters.WorldModel(),
            rdl_adapters.VisionBestBall(),
            )
        # extra column massaging
        # TODO: this is highly duplicate with analyze_localization, refactor the libraries?
        df = self.dataframe
        # smoothen velocity only, for more accuracy in finding stationary phases
        df = analyze_robotstate.lib.smoothen(df, ['vel_x', 'vel_y', 'vel_Rz'])
        # find stationary phases
        analyze_robotstate.calc_moving(df)
        # unwrap pos_Rz discontinuity
        analyze_robotstate.unwrap_Rz(df)
        # calculate and filter teleports (which heavily skew noise statistics)
        analyze_robotstate.calc_teleport(df)
        self.dataframe = df


class RDLDataHandle():
    def __init__(self, rdl, robot, tolerances, dumpdataframe=None):
        self.rdl = rdl
        self.robot = robot
        self.tolerances = tolerances
        self.analyzer = None
        self.dataframe = None
        self.dumpdataframe = dumpdataframe
        self.loadOK = False
    def initialize(self):
        # load RDL if not done already
        # allow three kind of arguments: None (handled above), rdl file name (string) or rdl frames (list)
        # this is useful when analyzing the same rdl with different robot/annotation/tolerances arguments
        if isinstance(self.rdl, str):
            sys.stdout.write("loading " + self.rdl + " ...")
            sys.stdout.flush()
            self.rdl = RDLFile(self.rdl)
            self.rdl.parseRDL()
            sys.stdout.write(" done (" + str(len(self.rdl.frames)) + " frames)\n")
            sys.stdout.flush()
        # setup analyzer object
        self.analyzer = WorldModelAnalyzer(self.rdl, self.robot)
        # create pandas dataframe
        d = RdlAnnotationDataFrame(self.rdl, self.robot)
        self.dataframe = d.dataframe
        self.loadOK = True
        # debugging: dump a dataframe as ASCII table to file
        if self.dumpdataframe:
            tfile = open(self.dumpdataframe, 'w')
            tfile.write(self.dataframe.to_string())
            tfile.close()
            print("file written: %s" % str(self.dumpdataframe))


class RDLDataHandleInitTestCase(unittest.TestCase):
    def __init__(self, dataHandle, **kwargs):
        unittest.TestCase.__init__(self)
        self.dataHandle = dataHandle
    def runTest(self):
        self.dataHandle.initialize()
    def id(self):
        # compatibility with rosunit xmlrunner, which prints as test name whatever comes after '.':
        #    (self._class, self._method) = test.id().rsplit(".", 1)
        return ".%s" % (type(self).__name__.replace('.runTest', ''))


################################################
# Specific annotation testcase implementations #
################################################

# calling convention: in yaml, test arguments must be named consistently (not per se in same order though)
# for example BallPossessionAnnotationTestCase
# -   type:                BallPossession
#     start:               19:25:22.3
#     end:                 19:25:22.6
#     robot:               3
#     hasBall:             true

class BallPossessionAnnotationTestCase(_BaseAnnotationTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        _BaseAnnotationTestCase.__init__(self, dataHandle, verbose, **kwargs)
    def runTest(self):
        assert(self.dataHandle.loadOK)
        # process arguments
        # (note: cannot do it at constructor, since RDL typically is not loaded at the time)
        self.ageStart = self.convertYamlTimestampToAge(self.kwargs['start'])
        self.ageEnd = self.convertYamlTimestampToAge(self.kwargs['end'])
        self.robot = self.kwargs['robot']
        self.expectedBallPossession = self.kwargs['hasBall']
        # run
        frames = self.dataHandle.analyzer.framesBetweenAge(self.ageStart, self.ageEnd)
        countBpMatch = 0
        countBpMismatch = 0
        for frame in frames:
            if self.robot in frame.data.keys():
                if "ROBOT_STATE" in frame.data[self.robot].keys():
                    robotState = frame.data[self.robot]["ROBOT_STATE"].value
                    currentHasBall = robotState[4]
                    countBpMatch += (currentHasBall == self.expectedBallPossession)
                    countBpMismatch += (currentHasBall != self.expectedBallPossession)
                    if currentHasBall != self.expectedBallPossession and self.verbose:
                        print("robot {} {} the ball at age={:.2f} ({})".format(self.robot, ["does not have", "has"][currentHasBall], frame.age, self.convertAgeToTimeString(frame.age)))
        # check counts
        self.assertEqual(countBpMismatch, 0)
        duration = frames[-1].age - frames[0].age
        minimumExpectedCount = duration * 5.0 # 5Hz is a safe lower bound (be robust for data loss / subsampling)
        self.assertGreater(countBpMatch, minimumExpectedCount)


class ObjectStatisticsTestCase(_BaseAnnotationTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        _BaseAnnotationTestCase.__init__(self, dataHandle, verbose, **kwargs)
        self.objectData = []
    def runTest(self):
        assert(self.dataHandle.loadOK)
        # process arguments
        self.ageStart = self.convertYamlTimestampToAge(self.kwargs['start'])
        self.ageEnd = self.convertYamlTimestampToAge(self.kwargs['end'])
        # analyze dataframe
        df = self.dataHandle.dataframe
        mask = (df['age'] >= self.ageStart) & (df['age'] <= self.ageEnd)
        df = df.loc[mask]
        dfStationary = df.loc[df['moving'] == False]
        dfDynamic = df.loc[df['moving'] == True]
        self.assertGreater(len(df), 0)
        for (name, sigmaTol, dynamic, expected, deltaTol) in self.objectData:
            v = df[name].values
            # check for missing (nan) data
            missingAge = list(df['age'].values[np.isnan(v)])
            if len(missingAge):
                self.assertEqual([], missingAge) # fail upon missing data
            s = name
            # optionally select stationary or dynamic subset
            if dynamic == False:
                v = dfStationary[name].values
                s += "(stationary)"
            if dynamic == True:
                v = dfDynamic[name].values
                s += "(dynamic)"
            s += " age=[{:.2f} .. {:.2f}]: ".format(self.ageStart, self.ageEnd)
            # continue if no data remains
            if len(v) == 0:
                s += "no data"
                if self.verbose:
                    print(s)
                continue
            # TODO optional tolerance distance scaling
            mean = np.mean(v)
            std = np.std(v)
            s += "n={} std={:.3f} mean={:.3f}".format(len(v), std, mean)
            if expected:
                delta = abs(mean - expected)
                s += " expected={:.3f} delta={:.3f}".format(expected, delta)
            # display, assert
            if self.verbose:
                print(s)
            self.assertLess(std, sigmaTol)
            if expected:
                self.assertLess(delta, deltaTol)


class StillRobotAnnotationTestCase(ObjectStatisticsTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        ObjectStatisticsTestCase.__init__(self, dataHandle, verbose, **kwargs)
        tol = self.dataHandle.tolerances['localization']['stationary']
        self.objectData.append(('pos_x', tol['xySigma'], None, kwargs['x'], tol['xyDelta']))
        self.objectData.append(('pos_y', tol['xySigma'], None, kwargs['y'], tol['xyDelta']))
        self.objectData.append(('pos_Rz', tol['rzSigma'], None, kwargs['Rz'], tol['rzDelta']))


class MovingRobotAnnotationTestCase(_BaseAnnotationTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        _BaseAnnotationTestCase.__init__(self, dataHandle, verbose, **kwargs)
    def runTest(self):
        assert(self.dataHandle.loadOK)
        # process arguments
        self.ageStart = self.convertYamlTimestampToAge(self.kwargs['start'])
        self.ageEnd = self.convertYamlTimestampToAge(self.kwargs['end'])
        # analyze dataframe
        df = self.dataHandle.dataframe
        mask = (df['age'] >= self.ageStart) & (df['age'] <= self.ageEnd)
        df = df.loc[mask]
        self.assertGreater(len(df), 0)
        maskNotMoving = df['moving'] == False
        df = df.loc[maskNotMoving]
        if len(df) and self.verbose:
            for age in df['age'].values:
                ts = self.convertAgeToTimeString(age)
                print("robot not moving at age={:.2f} ({})".format(age, ts))
        self.assertEqual(len(df), 0)


class StillObstacleAnnotationTestCase(_BaseAnnotationTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        _BaseAnnotationTestCase.__init__(self, dataHandle, verbose, **kwargs)
    def runTest(self):
        pass


class StillBallAnnotationTestCase(ObjectStatisticsTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        ObjectStatisticsTestCase.__init__(self, dataHandle, verbose, **kwargs)
        # stationary tests
        tol = self.dataHandle.tolerances['ball']['still']['stationary']
        self.objectData.append(('ball_x', tol['pos'], False, kwargs['x'], tol['delta']))
        self.objectData.append(('ball_y', tol['pos'], False, kwargs['y'], tol['delta']))
        self.objectData.append(('ball_vx', tol['vel'], False, 0.0, tol['delta']))
        self.objectData.append(('ball_vy', tol['vel'], False, 0.0, tol['delta']))
        # dynamic tests
        tol = self.dataHandle.tolerances['ball']['still']['dynamic']
        self.objectData.append(('ball_x', tol['pos'], True, kwargs['x'], tol['delta']))
        self.objectData.append(('ball_y', tol['pos'], True, kwargs['y'], tol['delta']))
        self.objectData.append(('ball_vx', tol['vel'], True, 0.0, tol['delta']))
        self.objectData.append(('ball_vy', tol['vel'], True, 0.0, tol['delta']))


class MovingBallAnnotationTestCase(_BaseAnnotationTestCase):
    def __init__(self, dataHandle, verbose, **kwargs):
        _BaseAnnotationTestCase.__init__(self, dataHandle, verbose, **kwargs)
    def runTest(self):
        pass


#######################
# Framework TestSuite #
#######################

class WorldModelRDLAnnotationTestSuite(unittest.TestSuite):
    def __init__(self, annotation, tolerances, rdl=None, robot=None, verbose=False, dumpdataframe=None):
        unittest.TestSuite.__init__(self)
        assert(isinstance(annotation, dict))
        assert(isinstance(tolerances, dict))
        # default: take rdl from annotation
        if rdl == None:
            rdl = annotation['rdl']
        # store arguments; loading RDL will be the first 'test'
        self.rdl = rdl
        if isinstance(self.rdl, str):
            if robot == None:
                robot = get_agent_from_rdl_filename(self.rdl)
        if robot == None:
            raise Exception("cannot guess robot from rdl -- robot argument is required")
        self.verbose = verbose
        self.robot = robot
        self.annotation = annotation
        self.tolerances = tolerances
        # setup
        self.dataHandle = RDLDataHandle(self.rdl, self.robot, self.tolerances, dumpdataframe) # lazy loader
        # generate a test per annotation
        self.generateTests()

    def generateTests(self):
        # test 0 will load RDL (if needed) and setup Analyzer
        self.addTest(RDLDataHandleInitTestCase(self.dataHandle))
        # generate a test per annotation
        count = 1
        for a in self.annotation['annotations']:
            handler = a['type'] + "AnnotationTestCase"
            del a['type'] # not needed anymore
            tcName = globals()[handler] # error upon missing implementation
            # construct the testcase, pass common data handle followed by specific yaml arguments
            tcInstance = tcName(self.dataHandle, self.verbose, **a)
            # store
            self.addTest(tcInstance)
            count += 1


############################
# Main and other utilities #
############################


def loadYaml(filename):
    f = open(filename, mode='r')
    y = yaml.load(f.read(), Loader=yaml.FullLoader)
    f.close()
    return y


def get_agent_from_rdl_filename(rdlfile):
    """
    Extract agent id as int from rdl file name.
    Example: /var/tmp/20191205_204236_r6.rdl will give output 6.
    """
    b, e = os.path.splitext(os.path.basename(rdlfile))
    parts = b.split("_")
    for p in parts:
        if p[0] == "r":
            try:
                return int(p[1:])
            except:
                pass
    return 0


if __name__ == '__main__':
    # handle arguments
    args = parse_arguments()
    annotation = loadYaml(args.annotation)
    rdlfile = annotation['rdl']
    if args.rdlfile: # overrule
        rdlfile = args.rdlfile
    robot = get_agent_from_rdl_filename(rdlfile)
    if args.robot: # overrule
        robot = args.robot
    # run
    c = WorldModelRDLAnnotationTestSuite(annotation, loadYaml(args.tolerances), rdlfile, robot, args.verbose, args.dumpdataframe)
    runner = unittest.TextTestRunner(verbosity=1+args.verbose)
    runner.run(c)

