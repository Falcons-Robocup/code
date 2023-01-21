# Copyright 2019-2022 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

from datetime import datetime
import sys, os
from shutil import copyfile
import subprocess
import pytz
import unittest
import falconspy

TEST_RDL_FILE = falconspy.FALCONS_DATA_PATH + '/internal/logfiles/20220626_rdltest.rdl'
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

        # Utilize the unittest's builtin diff mechanism when assertion fails
        self.maxDiff = None
        self.assertEqual(actual_output, expected_output, "output mismatch")

    def convert_to_localdate(self, date_str):
        fmt = "%Y-%m-%d,%H:%M:%S.%f"

        origin_tz = pytz.timezone("Europe/Amsterdam")
        origin_date = origin_tz.localize(datetime.strptime(date_str, fmt))

        local_date = origin_date.astimezone(tz=None)
        return local_date.strftime(fmt)


    def expected_rdlinfo(self):
        creation_date = self.convert_to_localdate("2022-06-26,16:06:29.763366")
        return f"""           filename: /var/tmp/20220626_160629_coach.rdl
           filesize: 3035400 [B] (2.89MB)
           hostname: bowser
           creation: {creation_date}
           duration: 69.27 [s]
          frequency: 50 [Hz]
         compressed: yes
        commit code: Coimbra2017-5993-gd56c5af2b
commit teamplayData: Coimbra2017-128-ge57b9af
          numFrames: 4380
        minDataSize: 11 [B]
        maxDataSize: 2242 [B]
        avgDataSize: 676.69 [B/frame]
       avgFrameSize: 693.01 [B/frame]
        avgOverhead: 16.33 [B/frame]
        avgDataRate: 41.78 [KB/s]
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
        self.run_compare_output(RDLFILTER, TEST_RDL_FILE + " " + TEST_RDL_FILE_TMP + " -t 1", expected_output)

        creation_date = self.convert_to_localdate("2022-06-26,16:06:29.763366")
        expected_output = f"""           filename: /var/tmp/20220626_160629_coach.rdl
           filesize: 3570 [B] (0.00MB)
           hostname: bowser
           creation: {creation_date}
           duration: 1.99 [s]
          frequency: 50 [Hz]
         compressed: yes
        commit code: Coimbra2017-5993-gd56c5af2b
commit teamplayData: Coimbra2017-128-ge57b9af
          numFrames: 97
        minDataSize: 11 [B]
        maxDataSize: 838 [B]
        avgDataSize: 19.53 [B/frame]
       avgFrameSize: 36.80 [B/frame]
        avgOverhead: 17.28 [B/frame]
        avgDataRate: 0.93 [KB/s]
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
        self.assertEqual(len(lines), 83086)
        self.assertEqual(lines[0], "frame=0/4380 age=0.069s")
        self.assertEqual(lines[1], "   0 L               CONFIG_EXECUTION -> {'frequency': 20.0, 'simulationSpeedupFactor': 1.0, 'tickFinishRtdbKey': 'SIMULATION_HEARTBEAT_DONE'}")
        self.assertEqual(lines[-1], "   5 L                   TP_HEARTBEAT -> 0")

    # TODO: test options of rdldump
    # TODO: test performance of rdldump and rdlfilter, these should not load entire file in memory ...

if __name__ == '__main__':
    # run
    unittest.main()
    #runner = unittest.TextTestRunner(verbosity=2)
    #result = runner.run('TestRdl')
    #sys.exit(len(result.errors))

