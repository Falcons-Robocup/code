""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
PKG='logging'
import roslib; roslib.load_manifest(PKG)

import sys, os
from shutil import copyfile
import subprocess
import unittest

TEST_RDL_FILE = '/home/robocup/falcons/data/internal/logfiles/20190618_rdltest.rdl'
TEST_RDL_FILE_TMP = '/var/tmp/rdltest.rdl'
RDLINFO = 'rosrun logging rdlinfo'
RDLFIX = 'rosrun logging rdlfix'
RDLFILTER = 'python /home/robocup/falcons/code/packages/facilities/logging/scripts/rdlFilter.py'
RDLDUMP = 'python /home/robocup/falcons/code/packages/facilities/logging/scripts/rdlDump.py'
# these ones are duplicate w.r.t. aliases ... TODO make common somehow? without paths?


class TestRdl(unittest.TestCase):

    def ensure_no_tmp(self):
        if os.path.isfile(TEST_RDL_FILE_TMP):
            os.remove(TEST_RDL_FILE_TMP)

    def run_get_output(self, tool, args):
        command = tool + ' ' + args
        print "running command: '{}'".format(command)
        actual_output = subprocess.check_output(command, shell=True)
        return actual_output
    
    def run_compare_output(self, tool, args, expected_output):
        actual_output = self.run_get_output(tool, args)
        self.assertEqual(actual_output, expected_output, "output mismatch: expected '\n{}\n', got '\n{}\n'".format(expected_output, actual_output))
    
    def expected_rdlinfo(self):
        return """     filename: /var/tmp/20190615_155806_coach.rdl
     filesize: 213781 [B] (0.20MB)
     hostname: bakpao
     creation: 2019-06-15,15:58:06.559372
     duration: 30.98 [s]
    frequency: 30 [Hz]
   compressed: yes
    numFrames: 60
  minDataSize: 2980 [B]
  maxDataSize: 3880 [B]
  avgDataSize: 3544.85 [B/frame]
 avgFrameSize: 3563.02 [B/frame]
  avgOverhead: 18.17 [B/frame]
  avgDataRate: 6.71 [KB/s]
"""

    def test_rdlinfo_help(self):
        # tools should always provide a help text
        actual_output = self.run_get_output(RDLINFO, "-h")
        self.assertTrue(len(actual_output) > 20)
        
    def test_rdlinfo(self):
        # rdlinfo utility
        expected_output = self.expected_rdlinfo()
        # note that the header of this datafile is incomplete: duration is 0.0 seconds, we will fix it in the next testcase
        expected_output = expected_output.replace('duration: 30.98 [s]', 'duration: 0.00 [s]')
        expected_output = expected_output.replace('  avgDataRate: 6.71 [KB/s]\n', '')
        self.run_compare_output(RDLINFO, TEST_RDL_FILE, expected_output)

    def test_rdlfix(self):
        # rdlfix utility: fix header (in particular the 'duration' field)
        # first copy file so we can modify it inline
        self.ensure_no_tmp()
        copyfile(TEST_RDL_FILE, TEST_RDL_FILE_TMP)
        expected_output = "" # rdlfix gives no output, it just fixes the file inline
        self.run_compare_output(RDLFIX, TEST_RDL_FILE_TMP, expected_output)
        # verify success by again running rdlinfo on the temporary file
        expected_output = self.expected_rdlinfo()
        self.run_compare_output(RDLINFO, TEST_RDL_FILE_TMP, expected_output)

    def test_rdlfilter_help(self):
        # tools should always provide a help text
        actual_output = self.run_get_output(RDLFILTER, "-h")
        self.assertTrue(len(actual_output) > 100)

    def test_rdlfilter_noarg(self):
        # rdlfilter: when no arguments are used, it shall generate the same file as original
        self.ensure_no_tmp()
        expected_output = ""
        self.run_compare_output(RDLFILTER, TEST_RDL_FILE + " " + TEST_RDL_FILE_TMP, expected_output)
        # verify success by again running rdlinfo on the temporary file
        # note that it automatically fixed header
        expected_output = self.expected_rdlinfo()
        
    def test_rdlfilter_arg(self):
        # rdlfilter: select only a small part of the file
        # the file was already reduced to only frames with age between 29 and 31 seconds
        # we select the last second
        self.ensure_no_tmp()
        expected_output = ""
        self.run_compare_output(RDLFILTER, TEST_RDL_FILE + " " + TEST_RDL_FILE_TMP + " -t 31", expected_output)
        expected_output = """     filename: /var/tmp/20190615_155806_coach.rdl
     filesize: 102892 [B] (0.10MB)
     hostname: bakpao
     creation: 2019-06-15,15:58:06.559372
     duration: 30.98 [s]
    frequency: 30 [Hz]
   compressed: yes
    numFrames: 30
  minDataSize: 2980 [B]
  maxDataSize: 3557 [B]
  avgDataSize: 3410.40 [B/frame]
 avgFrameSize: 3429.73 [B/frame]
  avgOverhead: 19.33 [B/frame]
  avgDataRate: 3.23 [KB/s]
"""
        self.run_compare_output(RDLINFO, TEST_RDL_FILE_TMP, expected_output)
        # TODO: revise header and the invariants related to it
        # * get rid of rdlfix, it will either not be needed or applied automatically
        # * first frame has age=0
        # * duration == age of last frame
        # * filename is redundant w.r.t. creation + hostname, so remove
        # * frequency needs to be calculated, not stored (it might become more dynamical, if we sync using waitForPut?)
        # * need new 't0' timestamp field, which is the timestamp associated to age=0
        #   (it might be different from creation in case rdlfilter was used)
        # * overhead might not really interesting in rdlinfo, better to focus on frameSize (rename from dataSize)

    def test_rdldump_help(self):
        # tools should always provide a help text
        actual_output = self.run_get_output(RDLDUMP, "-h")
        self.assertTrue(len(actual_output) > 100)

    def test_rdldump_noarg(self):
        actual_output = self.run_get_output(RDLDUMP, TEST_RDL_FILE)
        lines = actual_output.splitlines()
        self.assertEqual(len(lines), 4423)
        self.assertEqual(lines[0], "frame=0/60 age=29.011s")
        self.assertEqual(lines[1], "   0 S                          BALLS -> [[[-1.0473138093948364, -0.7850692272186279, 0.0], [0.8006411790847778, -1.1386703252792358, 0.0], 1.0, [1, 20254032]]]")
        self.assertEqual(lines[-1], "   5 L                   TP_HEARTBEAT -> 0")

    # TODO: test options of rdldump
    # TODO: test performance of rdldump and rdlfilter, these should not load entire file in memory ...
    
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_rdl', TestRdl)

