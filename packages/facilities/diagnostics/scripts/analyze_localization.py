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
    descriptionTxt = """Analyze localization noise levels from RDL file, write a brief report to stdout.
"""
    exampleTxt = """Examples:
    analyze_localization /var/tmp/20191205_204236_r6.rdl

    overall standard deviation (n=2657):
        x :  0.0107
        y :  0.0083
        Rz:  0.0042
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-p', '--plot', help='show a plot', action='store_true')
    parser.add_argument('-s', '--summary', help='only display overall summary', action='store_true')
    parser.add_argument('-a', '--all', help='show as much as possible', action='store_true')
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()

# TODO refactor to use newer Model from rdl_model.py
def dumpdf(df, filename='/tmp/dataframe.txt'):
    """
    Debugging: dump a dataframe as ASCII table to file.
    """
    tfile = open(filename, 'w')
    tfile.write(df.to_string())
    tfile.close()

def run(rdlfile, agent, makeplot=False):
    """
    Calculate dataframe for analyzing localization.
    """
    # get interesting data from RDL
    df = analyze_robotstate.make_dataframe(rdlfile, agent)
    # TODO: be more consistent when modifying dataframe inline or returning a modified copy ...
    df = analyze_robotstate.select_inplay(df)
    if df.empty:
        raise RuntimeError("No inplay data found.")
    # smoothen velocity only, for more accuracy in finding stationary phases
    df = analyze_robotstate.lib.smoothen(df, ['vel_x', 'vel_y', 'vel_Rz'])
    # find stationary phases
    analyze_robotstate.calc_moving(df)
    analyze_robotstate.lib.make_phases(df, lambda row: row['moving'] == False)
    # unwrap pos_Rz discontinuity
    analyze_robotstate.unwrap_Rz(df)
    # remove average position per phase to get noise levels, store the averages as side product
    df = analyze_robotstate.split_mean_noise_per_phase(df)
    # select data where robot is not moving for a sufficient amount of time
    df = analyze_robotstate.lib.select_phases_duration(df, minduration=4.0)
    # calculate and filter teleports (which heavily skew noise statistics)
    analyze_robotstate.calc_teleport(df)
    df = analyze_robotstate.lib.select_phases(df, lambda dfp: all(dfp['teleport'] == False))
    # also make sure we are only looking at inplay data
    df = analyze_robotstate.lib.select_phases(df, lambda dfp: all(dfp['inplay'] == True))
    dumpdf(df)
    if df.empty:
        raise RuntimeError("No stationary data found.")
    # output
    return df


def plot(df, showall=False):
    """
    Plot dataframe.
    Will make a subplot, aligned vertically, with selected noise on top and unfiltered data at the bottom.
    """
    if showall:
        dfn = df
    else:
        dfn = df.loc[df['select'] == True]
    fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)
    dof2col = {'x': 'blue', 'y': 'red', 'Rz': 'black'}
    for dof in ['x', 'y', 'Rz']:
        dfn.plot(kind='scatter', x='age', y='noise_'+dof, color=dof2col[dof], ax=axes[0], label=dof)
        df.plot(kind='line', x='age', y='pos_'+dof, color=dof2col[dof], ax=axes[1], label=dof)
    axes[0].set(xlabel='age', ylabel='noise (selection)')
    axes[0].legend()
    axes[0].grid(True)
    axes[1].set(xlabel='age', ylabel='position (unfiltered)')
    axes[1].legend()
    axes[1].grid(True)
    plt.show()


def print_phases(df, showall=False):
    """
    Display information per phase, in case it is still selected.
    Use option 'showall' to overrule selection.
    """
    phases = sorted(list(set(df['phase'].values)))
    if 0 in phases:
        phases.remove(0)
    # iterate over phases
    for phase in phases:
        # filter dataframes for this phase
        dfp = df[df['phase'] == phase]
        # check selection
        show = False
        if all(dfp['select']):
            show = True
        if showall:
            show = True
        # display?
        if show:
            duration = max(dfp['age']) - min(dfp['age'])
            ignore = ""
            if not all(dfp['select']):
                ignore = " ignored" # TODO: show reason? (duration too long, teleport, partial out-of-play)
            print("phase (start={:.1f}s, duration={:.1f}s, id={:d}){:s}:".format(min(dfp['age']), duration, phase, ignore))
            # show statistics
            dfs = dfp.std()
            for dof in ['x', 'y', 'Rz']:
                print("  {:>2s}: mean={:7.4f} std={:7.4f}".format(dof, dfp['mean_'+dof].values[0], dfs['pos_'+dof]))
            # teleport?
            if any(dfp['teleport']):
                print("      teleport detected")
    if not showall:
        print("")
        print("(omitted phases were filtered due to for example being too short in duration, or suffered from a teleport)")


def print_overall_std(df):
    # apply selection and calculate noise
    dfs = df.loc[df['select'] == True]
    dfss = dfs.std()
    n = dfs.count()['noise_x']
    print("")
    print("overall standard deviation (n={}):".format(n))
    print("   x: {:7.4f}".format(dfss['noise_x']))
    print("   y: {:7.4f}".format(dfss['noise_y']))
    print("  Rz: {:7.4f}".format(dfss['noise_Rz']))
    # report teleports on all data, not only noise selection
    num_teleports = sum(df['teleport'])
    print("")
    print("number of detected teleports: {:d}".format(num_teleports))
    showteleports = True
    if showteleports and (num_teleports > 0):
        idx_teleport = df[df['teleport'] == True].index
        # show surrounding rows, but not all columns, to keep it brief
        rows = []
        for idx in idx_teleport:
            for offset in [-1, 0, 1]:
                rows.append(idx+offset)
        rows = sorted(list(set(rows)))
        dft = df.loc[rows, ['age', 'pos_x', 'pos_y', 'pos_Rz', 'vel_x', 'vel_y', 'vel_Rz', 'teleport']]
        print(dft)


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
    if not args.summary:
        # useful for localization verification against e.g. laser pointer measurements?
        print_phases(df, showall=args.all)
    print_overall_std(df)
    # plot?
    if args.plot:
        plot(df, args.all)

