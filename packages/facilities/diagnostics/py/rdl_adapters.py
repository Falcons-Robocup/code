""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python
# a set of data adapters to pandas dataframe


from collections import OrderedDict
import math
from ball_data import BallMeasurement



def convertTimeStamp(rtdbTimestamp):
    return float(rtdbTimestamp[0]) + 1e-6 * float(rtdbTimestamp[1])


class Common():
    def __init__(self):
        self.columntypes = OrderedDict([('age', 'float')])
        self.handles = {} # no handles - 'age' is treated specially by make_dataframe()


class WorldModel():
    def __init__(self):
        self.columntypes = OrderedDict([
            ('inplay', 'bool'), ('hasball', 'bool'), ('wm_t', 'float'),
            ('pos_x', 'float'), ('pos_y', 'float'), ('pos_Rz', 'float'),
            ('vel_x', 'float'), ('vel_y', 'float'), ('vel_Rz', 'float'),
            ('robot_speed', 'float'), ('robot_stationary', 'bool'),
            ('ball_x', 'float'), ('ball_y', 'float'), ('ball_z', 'float'),
            ('ball_vx', 'float'), ('ball_vy', 'float'), ('ball_vz', 'float')])
        def handleRobotState(data, row, value):
            data['inplay'][row] = (value[0] == 2) # TODO enum
            data['wm_t'][row] = convertTimeStamp(value[1])
            data['hasball'][row] = value[4]
            data['pos_x'][row] = value[2][0]
            data['pos_y'][row] = value[2][1]
            data['pos_Rz'][row] = value[2][2]
            data['vel_x'][row] = value[3][0]
            data['vel_y'][row] = value[3][1]
            data['vel_Rz'][row] = value[3][2]
            # velocity interpretation
            data['robot_speed'][row] = math.sqrt(value[3][0]**2 + value[3][1]**2)
            data['robot_stationary'][row] = data['robot_speed'][row] < 0.1 and abs(data['vel_Rz'][row]) < 0.1
        def handleWmBall(data, row, value):
            if len(value):
                b = value[0]
                data['ball_x'][row] = b[0][0]
                data['ball_y'][row] = b[0][1]
                data['ball_z'][row] = b[0][2]
                data['ball_vx'][row] = b[1][0]
                data['ball_vy'][row] = b[1][1]
                data['ball_vz'][row] = b[1][2]
        self.handles = {'ROBOT_STATE': handleRobotState, 'BALLS': handleWmBall}


class VisionBestBall():
    def __init__(self):
        self.columntypes = OrderedDict([('ball_azimuth', 'float'), ('ball_elevation', 'float'), ('ball_radius', 'float')])
        def handleVisionBall(data, row, value):
            if len(value['balls']):
                b = value['balls'][0]['measurements'][0]['m'] # first in list should be newest valid measurement
                data['ball_azimuth'][row] = b[4] - math.pi * 0.5 # correct to align 0 with facing-forward
                data['ball_elevation'][row] = b[5]
                data['ball_radius'][row] = b[6]
        # use worldmodel diagnostics where bad data has been filtered
        self.handles = {'DIAG_WORLDMODEL_LOCAL': handleVisionBall}


class VisionLoc():
    def __init__(self):
        self.columntypes = OrderedDict([
            ('loc_t', 'float'), ('loc_x', 'float'), ('loc_y', 'float'), ('loc_Rz', 'float')])
        def handleLocCandidates(data, row, value):
            for candidate in value:
                data['loc_t'][row] = convertTimeStamp(candidate[1])
                data['loc_x'][row] = candidate[2]
                data['loc_y'][row] = candidate[3]
                data['loc_Rz'][row] = candidate[4]
        self.handles = {'LOCALIZATION_CANDIDATES': handleLocCandidates}


class ObjectCandidatesAround():
    def __init__(self, x, y, typename):
        self.x = x
        self.y = y
        self.typename = typename
        self.columntypes = OrderedDict([
            (typename+'_t', 'float'), (typename+'_azimuth', 'float'), (typename+'_elevation', 'float'), (typename+'_radius', 'float')])
        def candidateOk(candidate):
            # calculate FCS coordinate of measurement
            b = BallMeasurement(candidate)
            return abs(b.ballFcsX - self.x) < 1.0 and abs(b.ballFcsY - self.y) < 1.0
        def handleCandidates(data, row, value):
            for candidate in value:
                if candidateOk(candidate):
                    data[typename+'_t'][row] = convertTimeStamp(candidate[1])
                    data[typename+'_azimuth'][row] = candidate[4]
                    data[typename+'_elevation'][row] = candidate[5]
                    data[typename+'_radius'][row] = candidate[6]
        self.handles = {typename.upper()+'_CANDIDATES_FCS': handleCandidates}


class BallCandidatesAround(ObjectCandidatesAround):
    def __init__(self, x, y):
        ObjectCandidatesAround.__init__(self, x, y, "ball")
class ObstacleCandidatesAround(ObjectCandidatesAround):
    def __init__(self, x, y):
        ObjectCandidatesAround.__init__(self, x, y, "obstacle")


class Actuation():
    def __init__(self):
        self.columntypes = OrderedDict([
            ('velsp_x', 'float'), ('velsp_y', 'float'), ('velsp_Rz', 'float'),
            ('action', 'int')])
        def handleRobotVelocitySetpoint(data, row, value):
            data['velsp_x'][row] = value[0]
            data['velsp_y'][row] = value[1]
            data['velsp_Rz'][row] = value[2]
        def handleAction(data, row, value):
            data['action'][row] = value['action'] # TODO enum2string
        self.handles = {'ROBOT_VELOCITY_SETPOINT': handleRobotVelocitySetpoint, 'ACTION': handleAction}


class BallHandling():
    def __init__(self):
        self.columntypes = OrderedDict([('bh_left', 'float'), ('bh_right', 'float')])
        def handleBallHandling(data, row, value):
            data['bh_left'][row] = value['left']['angleFraction']
            data['bh_right'][row] = value['right']['angleFraction']
        self.handles = {'DIAG_BALLHANDLING': handleBallHandling}


# TODO: move RCS extension from intercept analysis to here? (Is it useful elsewhere?)

