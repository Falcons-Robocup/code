import subprocess

import falconspy

from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH



class SimulatorLibrary(object):

    def start_simulator(self):
        subprocess.Popen(['simStart', '-a'])

    def stop_simulator(self):
        subprocess.Popen(['simStop', '-k'])

    def minimum_score(self, team, expected_str):
        rtdb = RtDB2Store(RTDB2_DEFAULT_PATH, False)
        actual = rtdb.get(0, "MATCH_STATE").value["goalsOwn"]
        expected = int(expected_str)
        if actual < expected:
            raise AssertionError("Score too low. Actual score: {0}  Expected minimum score: {1}".format(actual, expected))

    def has_scored_once(self, team):
        rtdb = RtDB2Store(RTDB2_DEFAULT_PATH, False)
        actual_score = rtdb.get(0, "MATCH_STATE").value["goalsOwn"]
        if actual_score < 1:
            raise AssertionError("Not scored yet")

    def success(self):
         return 'PASS'
