""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3
#


import sys, copy
import argparse
import yaml
import falconspy
from rdlLib import RDLFile


AGENTS_IGNORE = [1] # the ones which do not have ballHandlers


def loadYAMLcalibration(yamlfile):
    f = open(yamlfile, mode='r')
    y = yaml.load(f.read())
    f.close()
    # only return the calibration section
    # convert from list to dict, which is more convenient
    return {cal['robotId']: cal for cal in y['calibration']}


def updateYAML(newCalibration, yamlfile):
    # to preserve formatting, ordering and comments, we just process the yaml line by line - parsing is easy enough
    f = open(yamlfile, mode='r')
    lines = f.readlines()
    f.close()
    # parse helpers
    def stripComment(line):
        p = line.split('#')
        return p[0]
    def getValue(line):
        p = line.split(':')
        return p[1]
    def replaceValue(line, newValue):
        newValue = int(newValue)
        p = line.split('#')
        comment = ""
        if len(p) > 1:
            comment = ' #' + p[1]
        p = line.split(':')
        key = p[0] + ':'
        return "%-14s %d%s" % (key, newValue, comment)
    # open in write mode
    f = open(yamlfile, mode='w')
    inCalibration = False
    arm = "leftArm"
    robotId = 0
    for line in lines:
        newLine = None
        if line.startswith('calibration'):
            inCalibration = True
        if inCalibration:
            if "robotId" in line:
                robotId = int(getValue(stripComment(line)))
            if "leftArm" in line:
                arm = "leftArm"
            if "rightArm" in line:
                arm = "rightArm"
            if "down:" in line and newCalibration.has_key(robotId):
                newLine = replaceValue(line, int(newCalibration[robotId][arm]['down'])) + '\n'
            if "up:" in line and newCalibration.has_key(robotId):
                newLine = replaceValue(line, int(newCalibration[robotId][arm]['up'])) + '\n'
        if newLine == None:
            f.write(line)
        else:
            f.write(newLine)
    f.close()
    

def analyzeRDL(rdlfile):
    # TODO: improve mem usage by rdlLib and remove the need for calling parseRDL()
    f = RDLFile(rdlfile)
    f.parseRDL()
    # helpers
    result = {}
    emptyCal = {'robotId': None, 'leftArm': {'down': 1e6, 'up': 0.0}, 'rightArm': {'down': 1e6, 'up': 0.0}}
    def processItem(agent, angleLeft, angleRight):
        # zero reading might be produced by peripheralsInterface at init time, ignore them
        if angleLeft == 0.0 or angleRight == 0.0: 
            return
        if not result.has_key(agent):
            result[agent] = copy.deepcopy(emptyCal)
        result[agent]['robotId'] = agent
        result[agent]['leftArm']['down']  = min(result[agent]['leftArm']['down'], angleLeft)
        result[agent]['leftArm']['up']    = max(result[agent]['leftArm']['up'], angleLeft)
        result[agent]['rightArm']['down'] = min(result[agent]['rightArm']['down'], angleRight)
        result[agent]['rightArm']['up']   = max(result[agent]['rightArm']['up'], angleRight)
    # work through the data
    robotsWithoutBall = set()
    robotsWithBall = set()
    for frame in f.frames:
        for agent in frame.data.keys():
            # store the ballHandler angles
            key = "BALLHANDLERS_FEEDBACK"
            if frame.data[agent].has_key(key):
                item = frame.data[agent][key]
                angleLeft = item.value[0]
                angleRight = item.value[1]
                processItem(agent, angleLeft, angleRight)
            # extra check on ball possession, since calibration only makes sense if the robot was
            # without ball for a while, AND with ball
            key = "ROBOT_STATE"
            if frame.data[agent].has_key(key):
                item = frame.data[agent][key]
                hasball = item.value[4]
                if hasball:
                    robotsWithBall.add(agent)
                else:
                    robotsWithoutBall.add(agent)
    # warn for bad data regarding ball possession
    for agent in result.keys():
        if agent in AGENTS_IGNORE:
            continue
        if agent not in robotsWithoutBall:
            print "WARNING: agent %d never lost the ball, so DOWN value is unreliable" % (agent)
        if agent not in robotsWithBall:
            print "WARNING: agent %d never got the ball, so UP value is unreliable" % (agent)
    return result


def report(currentCalibation, newCalibration):
    # helper
    def findAgent(agent, calibrationDict):
        if agent in calibrationDict.keys():
            return calibrationDict[agent]
        return None
    # for all agents
    for agent in range(1, 20):
        # ignore agent 1, keeper has no ballHandlers
        if agent in AGENTS_IGNORE:
            continue
        curCal = findAgent(agent, currentCalibation)
        newCal = findAgent(agent, newCalibration)
        if curCal != None or newCal != None:
            if curCal == None:
                print "WARNING: calibration data missing in yaml for agent " + str(agent)
            elif newCal == None:
                print "WARNING: data missing in RDL for agent " + str(agent)
            else:
                print agent
                for arm in ['leftArm', 'rightArm']:
                    for side in ['down', 'up']:
                        print '%15s : %4d -> %4d' % (arm + '.' + side, int(curCal[arm][side]), int(newCal[arm][side]))

    
def run(args):
    # which yaml file to read/write
    yamlfile = falconspy.FALCONS_CODE_PATH + "/config/BallHandling.yaml"
    # load yaml
    currentCalibation = loadYAMLcalibration(yamlfile)
    # analyze values from RDL, come up with new calibration
    newCalibration = analyzeRDL(args.rdlfile)
    # pretty-print the values for comparison
    report(currentCalibation, newCalibration)
    # update?
    if args.update:
        # strip bad data
        for agent in AGENTS_IGNORE:
            if newCalibration.has_key(agent):
                del newCalibration[agent]
        updateYAML(newCalibration, yamlfile)
        print "INFO: yaml configuration file has been updated"


if __name__ == '__main__':
    # Argument parsing.
    descriptionTxt = 'Suggest new values for ballHandler angle calibration based on .rdl file.\n'
    exampleTxt = 'Example: checkBhCalibration.py ~/falcons/matchLogs2020/practice_matches/20190925_204116_coach_VDL_half2.rdl\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-u', '--update', help='update the configuration yaml file', action='store_true')
    parser.add_argument('rdlfile', help='.rdl file to load')
    args       = parser.parse_args()
    # run
    run(args)

