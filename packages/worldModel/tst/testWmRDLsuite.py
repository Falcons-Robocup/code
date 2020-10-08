""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

import sys, os
import shutil, glob
import argparse
import yaml, uuid
import traceback
import subprocess
import unittest
unittest.TestLoader.sortTestMethodsUsing = None
from checkWmRDL import get_agent_from_rdl_filename, loadYaml, WorldModelRDLAnnotationTestSuite



def parse_arguments():
    parser = argparse.ArgumentParser(description="Test suite based on RDL stimulation and annotation file(s). An annotation refers to a RDL file. Each RDL file is run through the stimulator, after which checks are performed.")
    parser.add_argument('-d', '--directory', help='directory containing annotation files', type=str, default="annotations")
    parser.add_argument('-a', '--annotation', help='annotation yaml file to run', type=str)
    parser.add_argument('-n', '--nostimulate', help='do not stimulate, instead directly check rdl', action='store_true')
    parser.add_argument('-t', '--tolerances', help='tolerance specification yaml file', type=str, default='tolerances.yaml')
    parser.add_argument('-v', '--verbose', help='display extra details and statistics', action='store_true')
    args, unknown = parser.parse_known_args()
    return args


class CleanupTestCase(unittest.TestCase):
    def __init__(self, tmpdir):
        unittest.TestCase.__init__(self)
        # it is sometimes handy to be able to inspect result
        # so let's leave the directory available for a while instead of directly cleaning it up
        # in order to prevent /tmp from filling up (like on Jenkins), we need to cleanup oldest folders
        self.tmpdir = tmpdir
        self.cleanupImmediately = False
        self.numKeep = 10
    def id(self):
        # compatibility with rosunit xmlrunner, which prints as test name whatever comes after '.':
        #    (self._class, self._method) = test.id().rsplit(".", 1)
        return ".%s" % (type(self).__name__.replace('.runTest', ''))
    def runTest(self):
        if self.cleanupImmediately:
            shutil.rmtree(tmpdir)
        # create dict with timestamps as key and dirname as value
        logdirs = {}
        for dirname, dirnames, filenames in os.walk('/tmp'):
            for subdirname in dirnames:
                if "testWm" in subdirname:
                    t = os.path.getmtime(dirname + '/' + subdirname)
                    logdirs[t] = dirname + '/' + subdirname
        # remove oldest ones
        count = 0
        for key in reversed(sorted(logdirs.keys())):
            count += 1
            if count > self.numKeep:
                shutil.rmtree(logdirs[key])


class WorldModelRDLStimulationTestCase(unittest.TestCase):
    def __init__(self, rdlInput, rdlStim):
        unittest.TestCase.__init__(self, methodName='test_stimulation')
        self.rdlInput = rdlInput
        self.rdlStim = rdlStim
        self.rdlStimTxt = rdlStim + ".txt"
    def test_stimulation(self):
        assert("_r" in self.rdlInput) # stimulation requires local data -> RDL must be generated on robot
        assert(not os.path.isfile(self.rdlStim)) # output file will be generated soon by stimWorldModel script
        command = "stimWorldModel " + self.rdlInput
        print("running command: '{}'".format(command))
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        # store stimulator output in a similarly named tmp file
        with open(self.rdlStimTxt, "w") as text_file:
            text_file.write("%s" % output)
        print("output written to: '{}'".format(self.rdlStimTxt))
        assert(os.path.isfile(self.rdlStim)) # name of output should match


class WorldModelTracingRateTestCase(unittest.TestCase):
    def __init__(self, rdl):
        unittest.TestCase.__init__(self, methodName='test_tracing_rate')
        self.rdl = rdl
    def test_tracing_rate(self):
        # use rdlinfo utility to determine the duration of the RDL
        command = "frun logging rdlinfo " + self.rdl
        print("running command: '{}'".format(command))
        output = subprocess.check_output(command, shell=True).decode("utf-8")
        duration_seconds = 1.0
        for line in output.splitlines():
            if " duration:" in line:
                words = line.split()
                duration_seconds = float(words[1])
        # look inside newest logdir, where WM tracing was generated, count the number of zipped trace files
        newest_logdir = subprocess.check_output('newest_logdir.py', shell=True).decode("utf-8").strip()
        trace_files = glob.glob(newest_logdir + '/*.gz')
        MB_total = 10.5 * len(trace_files) # rough estimate
        # calculate rate and check
        max_allowed_MB_per_second = 2.0 # 2.0 MB/s == one 10MB gzip hickup per 5 seconds
        actual_MB_per_second = MB_total / duration_seconds
        self.assertLess(actual_MB_per_second, max_allowed_MB_per_second)


def makePathAbsolute(f):
    fi = f # backup
    if os.path.exists(f):
        return os.path.abspath(f)
    # interpret path relative to script location
    f = os.path.join(os.path.dirname(__file__), f)
    if os.path.exists(f):
        return os.path.abspath(f)
    raise Exception("cannot locate file/dir " + fi)


class WorldModelRDLTestSuite(unittest.TestSuite):
    """
    Setup the test suite, consisting of a sequence of tests. Also setup tmp directory, since it is referred to in test case definition (arguments).
    """
    def __init__(self, args = None):
        unittest.TestSuite.__init__(self)
        # default args?
        if args == None:
            args = parse_arguments()
        # resolve file paths
        args.directory = makePathAbsolute(args.directory)
        args.tolerances = makePathAbsolute(args.tolerances)
        # choose a temporary working folder
        tmpDirectory = "/tmp/testWm-" + str(uuid.uuid4()) + "/"
        os.mkdir(tmpDirectory)
        print("using temporary directory: " + tmpDirectory)
        # load tolerance file
        tolerances = loadYaml(args.tolerances)
        # determine list of annotation files to be processed
        if args.annotation: # overrule
            annotationFiles = [args.annotation]
        else:
            # default: scan directory
            annotationFiles = glob.glob(args.directory + '/*.yaml')
        # process all annotation files
        doStimulate = not args.nostimulate
        for annotationFile in annotationFiles:
            # load annotation
            try:
                annotation = loadYaml(annotationFile)
                rdlInput = os.path.expandvars(annotation['rdl'])
            except:
                traceback.print_exc()
                raise Exception("failed to load annotation " + annotationFile)
            # copy RDL
            rdlToInspect = os.path.join(tmpDirectory, os.path.basename(rdlInput))
            shutil.copyfile(rdlInput, rdlToInspect)
            # stimulate?
            if doStimulate:
                rdlStim = rdlToInspect.replace('.rdl', '_stim.rdl')
                self.addTest(WorldModelRDLStimulationTestCase(rdlToInspect, rdlStim))
                rdlToInspect = rdlStim
                self.addTest(WorldModelTracingRateTestCase(rdlStim))
            # check against annotation
            self.addTest(WorldModelRDLAnnotationTestSuite(annotation, tolerances, rdlToInspect, verbose=args.verbose, dumpdataframe=os.path.join(tmpDirectory, os.path.basename(annotationFile).replace('.yaml', '_dataframe.txt'))))
        # add cleanup action as final test
        self.addTest(CleanupTestCase(tmpDirectory))


if __name__ == '__main__':
    args = parse_arguments()
    # run
    runner = unittest.TextTestRunner(verbosity=1+args.verbose)
    result = runner.run(WorldModelRDLTestSuite(args))
    sys.exit(len(result.errors))

