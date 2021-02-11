# Copyright 2019 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import analyze_lib as lib
import numpy as np
from collections import OrderedDict



def make_dataframe(rdl, agent):
    # make sure we have rdl frames
    if isinstance(rdl, str):
        # load RDL from file
        rdlfile = rdl
        rdl = lib.RDLFile(rdlfile)
        rdl.parseRDL()
        rdlframes = rdl.frames
    elif isinstance(rdl, list):
        rdlframes = rdl
    else:
        raise RuntimeError("Don't know what to do with RDL argument")
    # setup for calling core frame parser
    # TODO: embed the 'select' column in analyze_lib instead of adding it here?
    columntypes = OrderedDict([
        ('age', 'float'), ('select', 'bool'), ('inplay', 'bool'), ('hasball', 'bool'),
        ('pos_x', 'float'), ('pos_y', 'float'), ('pos_Rz', 'float'),
        ('vel_x', 'float'), ('vel_y', 'float'), ('vel_Rz', 'float')])
    def handle(data, row, value):
        data['select'][row] = True
        data['inplay'][row] = (value[0] == 2) # TODO enum
        data['hasball'][row] = value[4]
        data['pos_x'][row] = value[2][0]
        data['pos_y'][row] = value[2][1]
        data['pos_Rz'][row] = value[2][2]
        data['vel_x'][row] = value[3][0]
        data['vel_y'][row] = value[3][1]
        data['vel_Rz'][row] = value[3][2]
    keyfunctions = {'ROBOT_STATE': handle}
    # construct the pandas DataFrame object
    df = lib.make_dataframe(rdlframes, agent, columntypes, keyfunctions)
    return df


def unwrap_Rz(df):
    """
    Remove discontinuities at between pos_Rz = 0 and 2pi.
    """
    mask = ~np.isnan(df['pos_Rz'])
    df.loc[mask, 'pos_Rz'] = np.unwrap(df.loc[mask, 'pos_Rz'])


def select_inplay(df):
    """
    Select data from frame where robot was inplay. Leaves input unchanged.
    """
    df['select'] &= (df['inplay'] == True)
    return df


def split_mean_noise_per_phase(df):
    """
    Calculate average position per phase and subtract it in order to estimate noise levels.
    Leaves input unchanged, returns dataframe with extra columns 'mean*' and 'noise*' contains the averages.
    """
    # which phases to keep
    phases = set(df['phase'].values)
    phases.remove(0)
    # iterate over phases
    result = df.copy()
    for phase in phases:
        # filter dataframe for this phase
        mask = (df['phase'] == phase)
        maskn = (df['phase'] == phase) & ~np.isnan(df['pos_Rz'])
        # if one coordinate is missing (NaN), then all are
        dfp = df[maskn]
        # subtract average and store it
        for dof in ['x', 'y', 'Rz']:
            avg = np.mean(dfp['pos_'+dof])
            result.loc[mask, 'noise_'+dof] = result.loc[mask, 'pos_'+dof] - avg
            result.loc[mask, 'mean_'+dof] = avg
            if dof == 'Rz':
                # numpy.unwrap was used to make angle continuous, but from now on (presentation purposes) we want a standardized value in [0,2pi)
                result.loc[mask, 'mean_'+dof] %= (2 * np.pi)
    return result


def calc_moving(df, thresholdXY=0.01, thresholdRz=0.01):
    """
    Modify dataframe by adding a column 'moving' to identify when robot was moving.
    (Best to first smoothen velocity data.)
    """
    def f(row):
        return (abs(row['vel_x']) > thresholdXY) or (abs(row['vel_y']) > thresholdXY) or (abs(row['vel_Rz']) > thresholdRz)
    df['moving'] = df.apply(f, axis=1)


def calc_teleport(df, thresholdXY=0.2, thresholdRz=0.5):
    """
    Modify dataframe by adding a column 'teleport' (boolean) to identify vision glitches.
    """
    delta_x = df['pos_x'] - np.roll(df['pos_x'], 1)
    delta_y = df['pos_y'] - np.roll(df['pos_y'], 1)
    delta_Rz = df['pos_Rz'] - np.roll(df['pos_Rz'], 1)
    df['teleport'] = (abs(delta_x) > thresholdXY)
    df['teleport'] |= (abs(delta_y) > thresholdXY)
    df['teleport'] |= (abs(delta_Rz) > thresholdRz)
    # fix edge
    df.loc[0, 'teleport'] = False
    # ignore going inplay
    df['teleport'] &= (np.roll(df['inplay'], 1) == True)


def calc_vel_rcs_raw(df):
    """
    Modify dataframe by adding column 'vel_*_rcs_raw' for velocity in RCS.
    (Might be handy to first smoothen pos_Rz, to reduce error propagation.)
    This velocity is going to be noisy still.
    """
    vx = df.get('vel_x').values
    vy = df.get('vel_y').values
    angle = np.pi*0.5 - df.get('pos_Rz').values
    s = np.sin(angle)
    c = np.cos(angle)
    df.insert(len(df.columns), 'vel_x_rcs_raw', c * vx - s * vy)
    df.insert(len(df.columns), 'vel_y_rcs_raw', s * vx + c * vy)
    # rotational velocity is the same in RCS as in FCS
    df.insert(len(df.columns), 'vel_Rz_rcs_raw', df.get('vel_Rz').values)
    return df


def calc_vel_rcs(df):
    """
    Modify dataframe by adding columns 'vel_*_rcs' smoothening '*_raw' data.
    """
    for dof in ['x', 'y', 'Rz']:
        df.insert(len(df.columns), 'vel_'+dof+'_rcs', df.get('vel_'+dof+'_rcs_raw').values)
    df = lib.smoothen(df, ['vel_x_rcs', 'vel_y_rcs', 'vel_Rz_rcs'], [10])
    # TODO apply a better (non-lagging) smoothener, like ewma or savitzky_golay or ..
    return df


def calc_acc_rcs(df):
    """
    Modify dataframe by adding acceleration columns 'acc_*_rcs'.
    """
    dt = df['age'] - np.roll(df['age'], 1)
    for dof in ['x', 'y', 'Rz']:
        v = df['vel_'+dof+'_rcs']
        acc = (v - np.roll(v, 1)) / dt
        df.insert(len(df.columns), 'acc_'+dof+'_rcs', acc)
    return df

