""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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
    
