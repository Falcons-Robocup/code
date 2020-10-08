""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python -u


import argparse
import analyze_lib as lib
import rdl_adapters
import math
import numpy as np
import traceback
from latency_model import LatencyModel



def parse_arguments():
    descriptionTxt = """Calibrate object (ball/obstacle) latency. See wiki calibration/latency-ball.
"""
    exampleTxt = """Example output: TODO
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-p', '--plot', help='show a plot of the modeled phases', action='store_true')
    parser.add_argument('-P', '--plotAll', help='show all data in plot', action='store_true')
    parser.add_argument('-d', '--dump', help='dump dataframe ASCII tables to /tmp', action='store_true')
    parser.add_argument('-m', '--mode', help='what to calibrate (which data to use)', type=str, default='ball', choices=['ball', 'obstacle'])
    parser.add_argument('-x', help='x position of the object', type=float, default=0.0)
    parser.add_argument('-y', help='y position of the object', type=float, default=2.0)
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()



class ObjectLatencyModel(LatencyModel):

    def __init__(self, *args, **kwargs):
        # interested in balls or obstacles?
        typename = "ball"
        if kwargs.has_key("typename"):
            typename = kwargs["typename"]
            del kwargs["typename"]
        pos = (0, 2)
        if kwargs.has_key("pos"):
            pos = kwargs["pos"]
            del kwargs["pos"]
        LatencyModel.__init__(self, *args, **kwargs) # load RDL etc
        self.makeDataFrame(rdl_adapters.WorldModel(), rdl_adapters.BallCandidatesAround(*pos), rdl_adapters.ObstacleCandidatesAround(*pos))
        self.minSpeed = 0.1
        self.minDuration = 1.0
        def selector(row):
            # phase selection: robot must be purely rotating
            return (abs(row['vel_Rz']) > self.minSpeed) and (abs(row['vel_y']) < 0.2) and (abs(row['vel_x']) < 0.2)
        lib.make_phases(self.dataframe, selector)
        self.dataframe = lib.select_phases_duration(self.dataframe, self.minDuration)
        # unwrap angles to prevent problems with discontinuities
        for col in ['pos_Rz', 'ball_azimuth', 'obstacle_azimuth']:
            mask = ~np.isnan(self.dataframe[col])
            self.dataframe.loc[mask, col] = np.unwrap(self.dataframe.loc[mask, col])
        # determine object placement offset
        mask = self.dataframe['robot_stationary'] & self.dataframe['inplay'] & self.dataframe[typename+'_t']
        dfs = self.dataframe[mask]
        mean_robot_Rz = np.nanmean(dfs['pos_Rz'].values)
        mean_ball_azimuth = math.pi - np.nanmean(dfs['ball_azimuth'].values)
        mean_obstacle_azimuth = math.pi - np.nanmean(dfs['obstacle_azimuth'].values)
        # correct columns to make them comparable with pos_Rz
        self.dataframe['ball_azimuth_corr'] = math.pi - self.dataframe['ball_azimuth'] - mean_ball_azimuth + mean_robot_Rz
        self.dataframe['obstacle_azimuth_corr'] = math.pi - self.dataframe['obstacle_azimuth'] - mean_obstacle_azimuth + mean_robot_Rz
        # vision data is timestamped when exporting from multiCam to worldModel
        # run applicable models
        if typename == "ball":
            self.model("ball_t", "ball_azimuth_corr", "wm_t", "pos_Rz")
        else:
            self.model("obstacle_t", "obstacle_azimuth_corr", "wm_t", "pos_Rz")


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name
    agent = lib.get_agent_from_rdl_filename(args.rdlfile)
    assert(agent > 0)
    # run
    try:
        m = ObjectLatencyModel(args.rdlfile, agent, resampleFrequency=10.0, typename=args.mode, pos=(args.x, args.y))
    except Exception as e:
        traceback.print_exc()
    # write details to file?
    if args.dump:
        m.dumpdf(m.dataframe, filename='/tmp/dataframe.txt')
    # display results
    m.printSummary()
    # plot?
    if args.plot or args.plotAll:
        m.plot(all=args.plotAll)

