# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3


import os, sys
import argparse
import time
import paramiko
import falconspy
import rtdb2tools




def parse_arguments():
    descriptionTxt = """Sync diagnostics file(s) from given robot to current devlaptop."""
    exampleTxt = "TODO"
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    robot = rtdb2tools.guessAgentId()
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=robot)
    parser.add_argument('-f', '--frequency', help='refresh frequency in Hz', type=float, default=1)
    parser.add_argument('-p', '--filepattern', help='file pattern to sync (newest only)', type=str, default="/tmp/diagnostics_r*.txt")
    # TODO: target option? so it can be run on robot
    return parser.parse_args()


class SyncDiagnostics():
    def __init__(self, robot, filepattern="/tmp/diagnostics_r*.txt", frequency=1.0):
        self.robot = robot
        self.frequency = frequency
        self.filepattern = filepattern
        self.file = None
    def resolve_file(self):
        # find newest file on robot matching given pattern
        self.file = None
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect('r' + str(self.robot))
        stdin, stdout, stderr = client.exec_command('cd / ; ls -1t {} | head -1'.format(self.filepattern))
        for line in stdout.read().splitlines():
            self.file = line
        if self.file == None:
            raise Exception("could not find newest diagnostics file on robot {} (pattern: '{}')".format(self.robot, self.filepattern))
        print("resolved file on robot {:d}: {}".format(self.robot, self.file))
    def run(self):
        dt = 1.0 / self.frequency
        self.resolve_file()
        while True:
            time.sleep(dt)
            self.iterate()
    def iterate(self):
        #cmd = "time rsync --update r{0:d}:{1} {1}".format(self.robot, self.file)
        t0 = time.time()
        cmd = "rsync -avz --append r{0:d}:{1} {1}".format(self.robot, self.file)
        os.system(cmd)
        elapsed = time.time() - t0
        print("elapsed: {:.3f}s".format(elapsed))


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    if args.robot == 0:
        raise Exception('could not guess robot id, please specify it')
    # run
    m = SyncDiagnostics(args.robot, args.filepattern, args.frequency)
    m.run()
    
