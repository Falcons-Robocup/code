""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python3


import os
import argparse
import falconspy
import analyze_robotstate
import matplotlib.pyplot as plt

import rtdb2tools


def parse_arguments():
    descriptionTxt = """Analyze motion performance from RDL file, write a brief report to stdout.
"""
    exampleTxt = """Examples:
    analyze_motion.py /var/tmp/20191205_204236_r6.rdl

TODO
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-p', '--plot', help='show a plot', action='store_true')
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()


def run(rdlfile, agent, makeplot=False):
    """
    Calculate dataframe for analyzing motion.
    """
    # get interesting data from RDL
    df = analyze_robotstate.make_dataframe(rdlfile, agent)
    df = analyze_robotstate.select_inplay(df)
    if df.empty:
        raise RuntimeError("No inplay data found.")
    # smoothen position Rz for more accurate velocity RCS transformation
    if 0:
        analyze_robotstate.unwrap_Rz(df)
        df = analyze_robotstate.lib.smoothen(df, ['pos_Rz'])
    # calculate unsmoothened velocity in RCS
    df = analyze_robotstate.calc_vel_rcs_raw(df)
    # smoothen velocity in RCS
    df = analyze_robotstate.calc_vel_rcs(df)
    # calculate acceleration in RCS based on smooth velocity
    df = analyze_robotstate.calc_acc_rcs(df)
    # output
    if df.empty:
        raise RuntimeError("No stationary data found.")
    return df


def plot(df):
    """
    Plot dataframe.
    """
    fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)
    dof2col = {'x': 'blue', 'y': 'red', 'Rz': 'black'}
    for dof in ['x', 'y', 'Rz']:
        df.plot(kind='line',    x='age', y='vel_'+dof+'_rcs', color=dof2col[dof], ax=axes[0], label=dof)
        df.plot(kind='scatter', x='age', y='vel_'+dof+'_rcs_raw', color=dof2col[dof], ax=axes[0])
        df.plot(kind='line',    x='age', y='acc_'+dof+'_rcs', color=dof2col[dof], ax=axes[1], label=dof)
    axes[0].set(xlabel='age', ylabel='velocity (RCS)')
    axes[0].legend()
    axes[0].grid(True)
    axes[1].set(xlabel='age', ylabel='acceleration (RCS)')
    axes[1].legend()
    axes[1].grid(True)
    plt.show()


def print_report(df):
    # apply selection and calculate noise
    print(df.describe())


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name
    agent = args.robot
    if agent == 0:
        agent = analyze_robotstate.lib.get_agent_from_rdl_filename(args.rdlfile)
    # run
    df = run(args.rdlfile, agent)
    # display results
    print_report(df)
    # TODO identify moving phases and distinguish acceleration from deceleration within. for piecewise reporting
    # TODO: better nan interpolation, sanity checks on dt, ..
    # plot?
    if args.plot:
        plot(df)

