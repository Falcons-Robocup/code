""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
PKG='worldModel'
import roslib; roslib.load_manifest(PKG)

import sys, os
import math
from shutil import copyfile
import subprocess
import unittest
from rdlLib import RDLFile
from WorldModelAnalyzer import WorldModelAnalyzer



# settings
TEST_RDL_FOLDER = '/home/robocup/falcons/data/internal/logfiles'
TEST_RDL_NAME = '20190615_115220_subset_r6.rdl'
AGENT = 6
TMP_FOLDER = '/var/tmp'
STIMULATE = 'stimWorldModel'

DO_STIMULATE = True
# False: run all checks on original file, showing original problems which have been solved afterwards
# True: run stimulator to apply worldModel to original data, replacing output, to verify progression


class TestWmRobot(unittest.TestCase):

    def ensure_no_tmp(self):
        if os.path.isfile(TEST_RDL_FILE_TMP):
            os.remove(TEST_RDL_FILE_TMP)

    @classmethod
    def run_get_output(self, tool, args):
        command = tool + ' ' + args
        print "running command: '{}'".format(command)
        output = subprocess.check_output(command, shell=True)
        return output
    
    ##### LOAD AND STIMULATE #####

    @classmethod
    def setUpClass(self):
        # copy RDL to tmp
        src = TEST_RDL_FOLDER + "/" + TEST_RDL_NAME
        tgt = TMP_FOLDER + "/" + TEST_RDL_NAME
        if src != tgt:
            copyfile(src, tgt)
        self.rdlfile = tgt
        if DO_STIMULATE:
            output = self.run_get_output(STIMULATE, self.rdlfile)
            # store output in a tmp file
            with open("/var/tmp/stimWorldModel_output.txt", "w") as text_file:
                text_file.write("%s" % output)
            self.rdlfile = tgt.replace('.rdl', '_stim.rdl')
        # load RDL file into memory, so checks can be performed
        self.rdl = RDLFile(self.rdlfile)
        self.rdl.parseRDL()
        # setup analyzer object
        self.ana = WorldModelAnalyzer(self.rdl, AGENT)
        
    ##### BALL TRACKING #####
    
    def test_ball_notmoving_robots_notmoving(self):
        # for a few seconds before kickoff, robots are looking from a distance at the ball
        data = self.ana.ballStatsBetweenAge(277.0, 279.0)
        print data
        # (conf: n=60, mean=1.00, std=0.00)
        # (x: n=60, mean=0.05, std=0.04)
        # (y: n=60, mean=-0.01, std=0.02)
        # (z: n=60, mean=0.00, std=0.00)
        # (vx: n=60, mean=-0.09, std=0.05)
        # (vy: n=60, mean=0.03, std=0.04)
        # (vz: n=60, mean=0.00, std=0.00)
        s = data.get('conf').stats()
        self.assertTrue(s.min > 0.95)
        s = data.get('x').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.05)
        s = data.get('y').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.05)
        s = data.get('z').stats()
        self.assertTrue(abs(s.mean) < 0.05)
        self.assertTrue(abs(s.sigma) < 0.05)
        s = data.get('vx').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.10)
        s = data.get('vy').stats()
        self.assertTrue(abs(s.mean) < 0.05)
        self.assertTrue(abs(s.sigma) < 0.05)
        s = data.get('vz').stats()
        self.assertTrue(abs(s.mean) < 0.05)
        self.assertTrue(abs(s.sigma) < 0.05)


    def test_ball_notmoving_robots_moving(self):
        # while positioning for kickoff, robots are moving while ball is still not
        data = self.ana.ballStatsBetweenAge(288.0, 295.0)
        print data
        # (conf: n=210, mean=1.00, std=0.00)
        # (x: n=210, mean=0.04, std=0.08)
        # (y: n=210, mean=0.02, std=0.07)
        # (z: n=210, mean=0.00, std=0.00)
        # (vx: n=210, mean=-0.00, std=0.12)
        # (vy: n=210, mean=-0.02, std=0.06)
        # (vz: n=210, mean=-0.00, std=0.00)
        s = data.get('conf').stats()
        #self.assertTrue(s.min > 0.80) # sensitive to loc? there is dip in confidence around age=290.4
        s = data.get('x').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.20)
        s = data.get('y').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.20)
        s = data.get('z').stats()
        self.assertTrue(abs(s.mean) < 0.05)
        self.assertTrue(abs(s.sigma) < 0.05)
        s = data.get('vx').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.20)
        s = data.get('vy').stats()
        self.assertTrue(abs(s.mean) < 0.10)
        self.assertTrue(abs(s.sigma) < 0.20)
        s = data.get('vz').stats()
        self.assertTrue(abs(s.mean) < 0.05)
        self.assertTrue(abs(s.sigma) < 0.05)
        
    ##### BALL POSSESSION #####

    def test_ball_possession_own(self):
        # get relevant frame sequence
        frames = self.ana.framesBetweenAge(298.0, 299.34)
        # check first frame has ball possession False
        frame = frames[0]
        robot = frame.data[AGENT]["ROBOT_STATE"].value
        hasball = robot[4]
        self.assertEqual(hasball, False)
        # find first frame where robot has ball possession
        for frame in frames:
            robot = frame.data[AGENT]["ROBOT_STATE"].value
            hasball = robot[4]
            if hasball:
                break
        self.assertEqual(hasball, True)
        expected_dribblestart = [robot[2][0], robot[2][1]] # x, y
        # iterate over frames
        for frame in frames:
            ball = frame.data[AGENT]["BALLS"].value[0]
            robot = frame.data[AGENT]["ROBOT_STATE"].value
            hasball = robot[4]
            if hasball:
                # ball position must be glued to robot, just check z coordinate and zero speed
                # dribble position is constant since ball pickup
                z = ball[0][2]
                vel = ball[1]
                conf = ball[2]
                bposs = ball[3]
                dribblestart = robot[5]
                self.assertAlmostEqual(z, 0.10)
                self.assertEqual(vel, [0.0, 0.0, 0.0])
                self.assertEqual(conf, 1.0)
                self.assertEqual(bposs, [2, 6])
                self.assertEqual(hasball, True)
                #self.assertEqual(dribblestart, expected_dribblestart) # TODO need accurate loc stimulation, which in turn needs a solution for getAndClear data gap..

    ##### OBSTACLES VERSUS TEAMMEMBERS #####
    def test_obstacles_versus_teammembers(self):
        frames = self.ana.framesBetweenAge(298.0, 299.34)
        # calculate distance of closest obstacle to teammembers
        closestDistance = 999
        for frame in frames:
            for obst in frame.data[AGENT]["OBSTACLES"].value:
                obstaclePos = obst[0]
                for agent in frame.data.keys():
                    if agent != AGENT: # ignore self
                        agentData = frame.data[agent]
                        if not agentData.has_key("ROBOT_STATE"):
                            continue
                        robotState = agentData["ROBOT_STATE"].value
                        teamMemberPos = robotState[2]
                        dxSquared = (obstaclePos[0] - teamMemberPos[0])**2
                        dySquared = (obstaclePos[1] - teamMemberPos[1])**2
                        distance = math.sqrt(dxSquared + dySquared)
                        if distance < closestDistance:
                            closestDistance = distance
                            print "robot r%d at (%.2f, %.2f) is %.2fm away from obstacle at (%.2f, %.2f)" % (agent, teamMemberPos[0], teamMemberPos[1], distance, obstaclePos[0], obstaclePos[1])
        # check
        self.assertGreater(closestDistance, 1.0)

    ##### OBSTACLE TRACKING #####

    def test_obstacles_robot_notmoving(self):
        # age=277 ==> t=2019-06-15,19:56:57.09
        data = self.ana.obstacleStatsBetweenAge(277.0, 279.0)
        for id in data.keys():
            print id
            print data[id]
        # expected results (meanX  meanY stdPos meanVel stdVel)
        expectedResults = [ (-2.03, -6.57,   0.1,   1.0,    0.3),
                            (-1.34, -3.86,   0.1,   1.0,    0.5), # noisy/erratic speed vector
                            (-0.09,  8.33,   0.1,   1.0,    0.5), # noisy/erratic speed vector
                            ( 2.86,  0.47,   0.1,   1.0,    0.5)] # noisy/erratic speed vector
        # check
        for d in data.values():
            s = d.get('x').stats()
            meanX = s.mean
            sigmaX = s.sigma
            s = d.get('y').stats()
            meanY = s.mean
            sigmaY = s.sigma
            s = d.get('vx').stats()
            meanVX = s.mean
            sigmaVX = s.sigma
            s = d.get('vy').stats()
            meanVY = s.mean
            sigmaVY = s.sigma
            # match
            closestDistance = 999
            for idx in range(len(expectedResults)):
                expected = expectedResults[idx]
                dxSquared = (expected[0] - meanX)**2
                dySquared = (expected[1] - meanY)**2
                distance = math.sqrt(dxSquared + dySquared)
                if distance < closestDistance:
                    closestDistance = distance
                    closestIdx = idx
            expected = expectedResults[closestIdx]
            print expected
            del expectedResults[closestIdx] # don't match again
            # verify
            self.assertLess(abs(meanX - expected[0]), 0.3)
            self.assertLess(abs(meanY - expected[1]), 0.3)
            self.assertLess(sigmaX, expected[2])
            self.assertLess(sigmaY, expected[2])
            self.assertLess(abs(meanVX), expected[3])
            self.assertLess(abs(meanVY), expected[3])
            self.assertLess(sigmaVX, expected[4])
            self.assertLess(sigmaVY, expected[4])


if __name__ == '__main__':
    # dev shortcut: run with given RDL file, only perform checks
    if len(sys.argv) == 2 and os.path.isfile(sys.argv[1]):
        f = os.path.abspath(sys.argv[1])
        TEST_RDL_FOLDER = os.path.dirname(f)
        TEST_RDL_NAME = os.path.basename(f)
        DO_STIMULATE = False
    # run
    import rosunit
    rosunit.unitrun(PKG, 'test_wm_robot', TestWmRobot)

