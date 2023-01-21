#!/usr/bin/env python3
#
# Jan Feitsma, december 2019
#
# automate the procedure of testing multiCam on a set of grabs
# (the procedure was something like: terminal 1 run cdrx, edit Makefile to select proper grab, make, in terminal 2 run cdmN where N is the appropriate robot, when done in terminal 1 run 'make kill')
#
# this script is useful when for instance developing dewarp or evaluating (dewarp) performance over time
# and perhaps even to demonstrate how the multiCam system works (how the tools depend on each other)
# maybe it can be used towards some kind of regression test suite


import sys, os
import signal
import glob
import argparse
import threading, time
import falconspy
import falconsrtdb

# setup RTDB
rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=True) # read mode



class TestGrabs():
    def __init__(self, verbose=True, worldmodelmode=False, dryrun=False):
        self.verbose = verbose
        self.dryrun = dryrun
        if dryrun:
            self.verbose = True
        self.worldmodelmode = worldmodelmode
        self.coderoot = falconspy.FALCONS_CODE_PATH
        self.grabroot = falconspy.FALCONS_DATA_PATH + '/internal/vision/multiCam'
        # setup signal handler for proper shutdown
        signal.signal(signal.SIGINT, self.signalHandler)

    def make(self, clean=False):
        make = 'make'
        if not self.verbose:
            make += ' -s'
        # raspiAnalyze
        self._chdir(self.coderoot + '/peripherals/raspiAnalyze/x86Build')
        if clean:
            self._cmd(make + ' clean')
        self._cmd(make + ' raspiAnalyze')
        # raspiSystem
        self._chdir(self.coderoot + '/peripherals/raspiSystem/x86Build')
        if clean:
            self._cmd(make + ' clean')
        self._cmd(make + ' raspiSystem')
        # jpgToRgb
        self._chdir(self.coderoot + '/tools/jpgToRgb')
        if clean:
            self._cmd(make + ' clean')
        self._cmd(make + ' jpgToRgb')
        # multiCam alone
        if not self.worldmodelmode:
            self._chdir(self.coderoot + '/packages/multiCam/alone')
            if clean:
                self._cmd(make + ' clean')
            self._cmd(make + ' multiCamAlone')
        # multiCam RTDB wrapper and worldModel, if requested
        pkgs = []
        if self.worldmodelmode:
            pkgs.append("multiCam")
            pkgs.append("worldModel")
            make = "fmake"
            if clean:
                make += ' --pre-clean'
            make += " " + " ".join(pkgs)
            self._cmd(make)

    def getGrabs(self, robot, timestamp):
        files = self.findGrabFiles(robot, timestamp)
        if len(files) == 0:
            raise Exception('could not find any matching grabs')
        elif len(files) != 4:
            raise Exception('expected 4 matching grabs, but got {}', len(files))
        files.sort()
        if self.verbose:
            print('using grabs:')
            for f in files:
                print("    " + f)
        return files

    def runJPG2RGB(self, grabs):
        self._chdir(self.coderoot + '/peripherals/raspiAnalyze/x86Build') # rgb files go here
        for cam in range(4):
            self._cmd(self.coderoot + '/tools/jpgToRgb/jpgToRgb -i {} -o cam{}Image.rgb'.format(grabs[cam], cam))

    def startRaspiAnalyze(self):
        self._chdir(self.coderoot + '/peripherals/raspiAnalyze/x86Build')
        for cam in range(4):
            self._cmd('nice -n 15 ./raspiAnalyze -l {}'.format(cam), bg=True)

    def startRaspiSystem(self):
        self._chdir(self.coderoot + '/peripherals/raspiSystem/x86Build')
        for cam in range(4):
            self._cmd('nice -n 15 ./raspiSystem -l {0} -g ../../raspiAnalyze/x86Build/cam{0}Image.rgb'.format(cam), bg=True)

    def startAloneM(self, robot, block):
        bg = not block
        self._chdir(self.coderoot + '/packages/multiCam/alone')
        self._cmd('make r{}'.format(robot), bg=bg) # also triggers make ... but well

    def startMulticamRTDB(self, robot, block):
        bg = not block
        self._chdir(self.coderoot + '/packages/multiCam')
        # start in background, so it does not block execution in testAllGrabs.py
        # run the RTDB observer at the same frequency as multiCam library
        # (see #define FPS in raspiAnalyze)
        self._cmd('export TURTLE5K_ROBOTNUMBER={0} ; frun multiCam multiCamNode -f 10.0 -i{0}'.format(robot), bg=bg)
        # TODO option -c ? ah well, it is nice to see the GUI flashing by

    def startWorldModel(self, robot, block):
        bg = not block
        self._chdir(self.coderoot + '/packages/worldModel')
        self._cmd('export TURTLE5K_ROBOTNUMBER={0} ; frun worldModel worldModelNode --inplay'.format(robot), bg=bg)

    def run(self, robot, grabs, block=True):
        # convert grabs to .rgb files
        self.runJPG2RGB(grabs)
        # start raspiAnalyze and raspiSystem 4x in background
        self.startRaspiAnalyze()
        self.startRaspiSystem()
        # start main multiCam process
        if self.worldmodelmode:
            self.startWorldModel(robot, False) # only multiCam will block
            self.startMulticamRTDB(robot, block)
        else:
            self.startAloneM(robot, block)

    def selectRobot(self):
        r = None
        while r == None:
            s = input("enter robot number: ")
            try:
                r = int(s)
            except:
                print("ERROR: could not convert to int, try again")
                continue
        return r

    def findGrabFiles(self, robot, timestamp = None):
        # allow 'grab' and 'cam' prefix, we have both in the data repo
        # allow grabs to be in subfolders
        files1 = glob.glob(self.grabroot + '/r{}/**/cam?_*.jpg'.format(robot), recursive=True)
        files2 = glob.glob(self.grabroot + '/r{}/**/grab?_*.jpg'.format(robot), recursive=True)
        result = sorted(files1 + files2)
        if timestamp != None:
            # filter on timestamp substring
            result = [f for f in result if timestamp in f]
        return result

    def findGrabTimestamps(self, robot):
        files = self.findGrabFiles(robot)
        t = []
        for f in files:
            a = f.replace('.jpg', '').split('_')
            t.append(a[-2] + '_' + a[-1])
        return sorted(list(set(t)))

    def selectTimestamp(self, robot):
        choices = self.findGrabTimestamps(robot)
        if len(choices) == 0:
            raise Exception('could not find any matching grabs')
        c = None
        while c == None:
            # show the choices
            for it in range(len(choices)):
                print("{0:3d}: {1:s}".format(it, choices[it]))
            s = input("select timestamp string: ")
            try:
                c = int(s)
                assert(c >= 0)
                assert(c < len(choices))
            except:
                print("ERROR: invalid input, try again")
                c = None
        return choices[c]

    # miscellaneous
    def shutdown(self, exit=True):
        self._cmd('make -s kill', self.coderoot + '/peripherals/raspiAnalyze/x86Build')
        if self.worldmodelmode:
            self._cmd('killall -q worldModelNode || true')
            self._cmd('killall -q multiCamNode || true')
        else:
            self._cmd('killall -q multiCamAlone || true')
        if exit:
            sys.exit(0)
    def signalHandler(self, signal, frame):
        self.shutdown(True)
    def _chdir(self, d):
        if self.verbose:
            print('cd {}'.format(d))
        if not self.dryrun:
            os.chdir(d)
    def _cmd(self, cmd, chdir=None, bg=False):
        if chdir != None:
            self._chdir(chdir)
        if self.verbose:
            print('command: {}'.format(cmd))
        else:
            cmd += ' >/dev/null'
        if bg:
            cmd += ' &'
        if not self.dryrun:
            r = os.system(cmd)
            if self.verbose:
                print('command exited with value: {}'.format(r))
            if r != 0:
                print('ERROR ({}) when executing command: {}'.format(r, cmd))
                self.shutdown()
                sys.exit(1)


def report(robot):
    # helper to inspect RTDB contents
    def get(key):
        item = rtdbStore.get(robot, key)
        if item != None:
            return item.value
        return None
    def formatter(arg):
        # for a list, return its size
        try:
            return str(len(arg))
        except:
            pass
        # otherwise convert to string
        return str(arg)
    # inspect robot position
    r = get("ROBOT_STATE")
    if r == None or r[0] != 2:
        locStr = "n/a                 "
    else:
        (x,y,Rz) = r[2]
        locStr = "({:5.2f},{:5.2f},{:6.3f})".format(x, y, Rz)
    # inspect balls
    balls = get("BALLS")
    if balls == None or len(balls) == 0:
        ballStr = "n/a                "
    else:
        (x,y,z) = balls[0][0]
        ballStr = "({:5.2f},{:5.2f},{:5.2f})".format(x, y, z)
    # inspect obstacles
    obstacles = get("OBSTACLES")
    obstacleStr = formatter(obstacles)
    # inspect ball possession (vision only)
    possessionStr = formatter(get("VIS_BALL_POSSESSION"))
    # report into a single line
    return "robotpos={} ball={} #obst={:2s} bposs={}".format(locStr, ballStr, obstacleStr, possessionStr)


def monitor(robot):
    frequency = 10.0
    dt = 1.0 / frequency
    active = False
    while True:
        # wait until the first data comes in
        item = rtdbStore.get(robot, "ROBOT_STATE", timeout=dt*2)
        if item != None:
            active = True
        else:
            # stop in case data grows stale
            if active:
                break
        if active:
            print(report(robot))
        time.sleep(dt)

def startMonitor(robot):
    t = threading.Thread(target=monitor, args=(robot,))
    t.start()



if __name__ == '__main__':
    # Argument parsing.
    descriptionTxt = 'Start the multiCam software using a set of grabs, automating all relevant commands. If robot or timestamp argument is omitted, a prompt will help to select it.\n\nTo learn more about grabs, check the wiki page:\nhttps://git.falcons-robocup.nl/falcons/code/-/wikis/diagnostics/grabs\n'
    exampleTxt = 'Examples:\n   testGrabs.py\n   testGrabs.py -r 5 -t 20191219_210335 -v -M\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int)
    parser.add_argument('-t', '--timestamp', help='timestamp string to use', type=str)
    parser.add_argument('-v', '--verbose', help='print each command/action', action='store_true')
    parser.add_argument('-d', '--dryrun', help='print each command/action without executing it', action='store_true')
    parser.add_argument('-m', '--make', help='also make the tools', action='store_true')
    parser.add_argument('-M', '--cleanmake', help='also clean make the tools', action='store_true')
    parser.add_argument('-W', '--worldmodel', help='run rtdb binaries multiCam and worldModel', action='store_true')
    args       = parser.parse_args()

    # setup TestGrabs object
    t = TestGrabs(args.verbose, args.worldmodel, args.dryrun)

    # determine set of grabs
    # if argument is not provided, provide a selection prompt
    if args.robot == None:
        robot = t.selectRobot()
    else:
        robot = args.robot
    if args.timestamp == None:
        timestamp = t.selectTimestamp(robot)
    else:
        timestamp = args.timestamp
    grabs = t.getGrabs(robot, timestamp)

    # run
    if args.make or args.cleanmake:
        t.make(args.cleanmake)
    if args.worldmodel and not args.dryrun:
        startMonitor(robot)
    t.run(robot, grabs, block=True)
    t.shutdown()
