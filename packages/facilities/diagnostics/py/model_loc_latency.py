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
import numpy as np
import rdl_adapters
from latency_model import LatencyModel



def parse_arguments():
    descriptionTxt = """Calibrate localization latency. See wiki calibration/latency-loc.
"""
    exampleTxt = """Example output: TODO
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-p', '--plot', help='show a plot', action='store_true')
    parser.add_argument('-P', '--plotAll', help='show all data in plot', action='store_true')
    parser.add_argument('-d', '--dump', help='dump dataframe ASCII tables to /tmp', action='store_true')
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()



class LocLatencyModel(LatencyModel):

    def __init__(self, *args, **kwargs):
        LatencyModel.__init__(self, *args, **kwargs) # load RDL etc
        self.makeDataFrame(rdl_adapters.WorldModel(), rdl_adapters.VisionLoc())
        # unwrap angles to prevent problems with discontinuities
        for col in ['pos_Rz', 'loc_Rz']:
            mask = ~np.isnan(self.dataframe[col])
            self.dataframe.loc[mask, col] = np.unwrap(self.dataframe.loc[mask, col])
        # setpoint duration is typically 1 second
        self.minSpeed = 0.3
        self.minDuration = 0.5
        def selector(row):
            # phase selection: robot must be driving purely in x (FCS)
            return (abs(row['vel_Rz']) > self.minSpeed) and (abs(row['vel_x']) < 0.2) and (abs(row['vel_y']) < 0.2)
        lib.make_phases(self.dataframe, selector)
        self.dataframe = lib.select_phases_duration(self.dataframe, self.minDuration)
        # vision data is timestamped when exporting from multiCam to worldModel
        # during the driving phases, vision weight factor should be set to zero, so worldModel is pure encoder
        self.model("loc_t", "loc_Rz", "wm_t", "pos_Rz")
        # TODO: calculate extra outputs
        #print "encoder drift: %.1f%%" % (int(1e2 * (abs(fitEncoder[0]) - fitVision[0])))



if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name
    agent = lib.get_agent_from_rdl_filename(args.rdlfile)
    assert(agent > 0)
    # run
    m = LocLatencyModel(args.rdlfile, agent, resampleFrequency=25.0)
    # write details to file?
    if args.dump:
        m.dumpdf(m.dataframe, filename='/tmp/dataframe.txt')
    # display results
    m.printSummary()
    # plot?
    if args.plot or args.plotAll:
        m.plot(all=args.plotAll)

