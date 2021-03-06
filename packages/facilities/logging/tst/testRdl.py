# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

import sys, os
from shutil import copyfile
import subprocess
import unittest
import falconspy

TEST_RDL_FILE = falconspy.FALCONS_DATA_PATH + '/internal/logfiles/20190618_rdltest.rdl'
TEST_RDL_FILE_TMP = '/var/tmp/rdltest.rdl'
RDLINFO = 'frun logging rdlinfo'
RDLFILTER = 'python3 ' + falconspy.FALCONS_CODE_PATH + '/packages/facilities/logging/scripts/rdlFilter.py'
RDLDUMP = 'python3 ' + falconspy.FALCONS_CODE_PATH + '/packages/facilities/logging/scripts/rdlDump.py'


class TestRdl(unittest.TestCase):

    def ensure_no_tmp(self):
        if os.path.isfile(TEST_RDL_FILE_TMP):
            os.remove(TEST_RDL_FILE_TMP)

    def run_get_output(self, tool, args):
        command = tool + ' ' + args
        print("running command: '{}'".format(command))
        actual_output = subprocess.check_output(command, shell=True)
        return actual_output.decode('utf-8')

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
        self.run_compare_output(RDLINFO, TEST_RDL_FILE, expected_output)

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
     creation: 2019-06-15,15:58:36.559372
     duration: 0.98 [s]
    frequency: 30 [Hz]
   compressed: yes
    numFrames: 30
  minDataSize: 2980 [B]
  maxDataSize: 3557 [B]
  avgDataSize: 3410.40 [B/frame]
 avgFrameSize: 3429.73 [B/frame]
  avgOverhead: 19.33 [B/frame]
  avgDataRate: 102.19 [KB/s]
"""
        self.run_compare_output(RDLINFO, TEST_RDL_FILE_TMP, expected_output)
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
    # run
    unittest.main()
    #runner = unittest.TextTestRunner(verbosity=2)
    #result = runner.run('TestRdl')
    #sys.exit(len(result.errors))

