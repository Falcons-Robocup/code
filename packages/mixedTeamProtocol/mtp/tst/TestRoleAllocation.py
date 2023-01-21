# Copyright 2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

import os
import traceback
import subprocess
import unittest



# path switching for out-of-source builds, assuming they change into the same directory as the binary
if os.path.isfile('testRoleAllocation'):
    CMD_NAME = './testRoleAllocation'
else:
    # MTP repo situation
    CMD_NAME = os.path.realpath(__file__) + '/../../build/mtp/testRoleAllocation'


def run_cmd(args):
    """Helper function to invoke the command-line interface."""
    cmd = CMD_NAME + " " + " ".join(args)
    output = subprocess.check_output(cmd, shell=True).decode("utf-8")
    return output


class RoleAllocationTestCase(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.maxDiff = None
    def assertMultiLineEqualNoWS(self, got, expected):
        linesGot = [line.strip() for line in got.strip().splitlines()]
        linesExpected = [line.strip() for line in expected.strip().splitlines()]
        self.assertListEqual(linesGot, linesExpected)


class TestNoArguments(RoleAllocationTestCase):
    def test(self):
        # setup
        args = []
        # run
        output = run_cmd(args)
        # check
        expected = """Running algorithm ... done ...
Result code: 0
Result allocation:
  [self] vendor=1  shirt=1  team=A hash=1     : DEFENDER_GENERIC                        
         vendor=1  shirt=2  team=A hash=2     : DEFENDER_MAIN                           
         vendor=1  shirt=3  team=A hash=3     : ATTACKER_ASSIST                         
         vendor=1  shirt=4  team=A hash=4     : ATTACKER_MAIN                           
         vendor=1  shirt=5  team=A hash=5     : GOALKEEPER          """
        self.assertEqual(output.strip(), expected.strip())


class TestCurrentGoalie(RoleAllocationTestCase):
    def test(self):
        self.maxDiff = None
        # setup
        args = []
        # run
        output = run_cmd(["-c", "GOALKEEPER"])
        # check
        expected = """Running algorithm ... done ...
Result code: 0
Result allocation:
  [self] vendor=1  shirt=1  team=A hash=1     : GOALKEEPER          (current=GOALKEEPER)
         vendor=1  shirt=2  team=A hash=2     : DEFENDER_GENERIC                        
         vendor=1  shirt=3  team=A hash=3     : DEFENDER_MAIN                           
         vendor=1  shirt=4  team=A hash=4     : ATTACKER_ASSIST                         
         vendor=1  shirt=5  team=A hash=5     : ATTACKER_MAIN       """
        self.assertEqual(output.strip(), expected.strip())


class TestCurrentDefender(RoleAllocationTestCase):
    def test(self):
        # setup
        args = []
        # run
        output = run_cmd(["-c", "DEFENDER_MAIN"])
        # check
        expected = """Running algorithm ... done ...
Result code: 0
Result allocation:
  [self] vendor=1  shirt=1  team=A hash=1     : DEFENDER_MAIN       (current=DEFENDER_MAIN)
         vendor=1  shirt=2  team=A hash=2     : DEFENDER_GENERIC                        
         vendor=1  shirt=3  team=A hash=3     : ATTACKER_ASSIST                         
         vendor=1  shirt=4  team=A hash=4     : ATTACKER_MAIN                           
         vendor=1  shirt=5  team=A hash=5     : GOALKEEPER          """
        self.assertEqual(output.strip(), expected.strip())


class TestNewPreference(RoleAllocationTestCase):
    """
    Taking over the goalkeeper role by setting preference.
    """
    def test(self):
        # setup
        args = []
        # run
        output = run_cmd(["-i 4", "-n 4", "-1 GOALKEEPER", "-p GOALKEEPER"])
        # check
        expected = """Running algorithm ... done ...
Result code: 0
Result allocation:
         vendor=1  shirt=1  team=A hash=1     : ATTACKER_MAIN       (current=GOALKEEPER)
         vendor=1  shirt=2  team=A hash=2     : DEFENDER_MAIN                           
         vendor=1  shirt=3  team=A hash=3     : ATTACKER_ASSIST                         
  [self] vendor=1  shirt=4  team=A hash=4     : GOALKEEPER          (preferred=GOALKEEPER)"""
        self.assertEqual(output.strip(), expected.strip())


class TestThreePlayers(RoleAllocationTestCase):
    def test(self):
        # setup
        args = []
        # run
        output = run_cmd(["-n", "3"])
        # check
        expected = """Running algorithm ... done ...
Result code: 0
Result allocation:
  [self] vendor=1  shirt=1  team=A hash=1     : DEFENDER_MAIN            
         vendor=1  shirt=2  team=A hash=2     : ATTACKER_MAIN            
         vendor=1  shirt=3  team=A hash=3     : GOALKEEPER          """
        self.assertMultiLineEqualNoWS(output, expected)


if __name__ == '__main__':
    # run
    unittest.main()

