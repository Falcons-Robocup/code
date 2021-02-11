# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
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

