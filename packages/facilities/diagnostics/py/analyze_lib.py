""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python


import os, sys
import traceback
from collections import OrderedDict, defaultdict
import pandas as pd
import numpy as np
import falconspy
from rdlLib import RDLFile, RDLResample


pd.set_option('display.float_format', lambda x: '%.4f' % x)



def get_agent_from_rdl_filename(rdlfile):
    """
    Extract agent id as int from rdl file name.
    Example: /var/tmp/20191205_204236_r6.rdl will give output 6.
    """
    b, e = os.path.splitext(os.path.basename(rdlfile))
    parts = b.split("_")
    for p in parts:
        if p[0] == "r":
            try:
                return int(p[1:])
            except:
                pass
    return 0


def smoothen(df, columns, args = [5]):
    """
    Quick and dirty smoothener on specified columns using pandas rolling_mean. Leaves input unchanged.
    """
    # backup data
    columns_keep = [c for c in df.columns if c not in columns]
    data_keep = {col: df[col] for col in columns_keep}
    # apply filter
    if len(args) < 1:
        raise RuntimeError("Require at least one argument")
    result = df.rolling(*args).mean()
    # restore data which was not to be overwritten
    for col in columns_keep:
        result[col] = data_keep[col]
    return result


def make_phases(df, predicate, name='phase'):
    """
    Insert a column 'phase' based on given predicate.
    Consecutive rows which satisfy given predicate are numbered.
    Example:
        predicate   : lambda(n): iseven(n)
        input       : 1 2 3 4 2 6 5 1 9 2 2
        output phase: 0 1 0 2 2 2 0 0 0 3 3
    """
    # helper to keep track of current phase
    class f:
        def __init__(self):
            self.phase = 0
            self.current = False
        def __call__(self, row):
            p = predicate(row)
            if p:
                if self.current:
                    return self.phase
                else:
                    self.current = True
                    self.phase += 1
                return self.phase
            self.current = False
            return 0
    df['phase'] = df.apply(f(), axis=1)


def select_phases_duration(df, minduration=1.0):
    # which phases to keep
    phases = set(df['phase'].values)
    if 0 in phases:
        phases.remove(0)
    # build result
    result = df.copy()
    for phase in phases:
        # filter dataframe for this phase
        mask = df['phase'] == phase
        dfp = df[mask]
        # calculate duration
        duration = max(dfp['age']) - min(dfp['age'])
        if duration < minduration:
            result.loc[mask, 'phase'] = 0
    # done
    return result


def select_phases(df, predicate=None):
    # which phases to keep
    phases = set(df['phase'].values)
    if 0 in phases:
        phases.remove(0)
    # build result
    result = df.copy()
    for phase in phases:
        # filter dataframe for this phase
        mask = df['phase'] == phase
        dfp = df[mask]
        # call given predicate function
        if not predicate(dfp):
            result.loc[mask, 'phase'] = 0
    # done
    return result


def make_dataframe(rdlframes, agent, columntypes, keyfunctions, strict=True):
    """
    Create a pandas.DataFrame object from a series of RDL frames.
    Use frame age as timestamp.
    Require a specification of column types (bool, float). Best to use an ordered dict, because its column ordering affects the ordering in DataFrame.
    Key functions must be a dict with handlers to parse specific data.
    """
    # require age
    assert('age' in columntypes.keys())
    assert(columntypes['age'] == 'float')
    # pre-allocate data
    columns = columntypes.keys()
    nrow = len(rdlframes)
    initial = {}
    initial['float'] = np.array([np.nan] * nrow, dtype='float64') # 64bits needed for dealing with timestamps
    initial['bool'] = np.array([False] * nrow, dtype='bool')
    initial['int'] = np.array([-1] * nrow, dtype='int')
    initial['str'] = np.array([""] * nrow, dtype=object)
    data = {col: initial[columntypes[col]].copy() for col in columns}
    # fill data vectors
    keys = keyfunctions.keys()
    row = 0
    count = 0
    blacklist = defaultdict(lambda: False)
    for frame in rdlframes:
        data['age'][row] = frame.age
        if agent in frame.data:
            agentData = frame.data[agent]
            for key in list(keys):
                if key in agentData and not blacklist[key]:
                    item = agentData[key] # type: RtDBFrameItem
                    f = keyfunctions[key]
                    if strict:
                        f(data, row, item.value)
                    else:
                        # allow for failures, but blacklist key at once
                        try:
                            f(data, row, item.value)
                        except Exception as e:
                            traceback.print_exc()
                            print("item.value:")
                            print(item.value)
                            print("blacklisting key " + key)
                            blacklist[key] = True
                    count += 1
        row += 1
    if count == 0:
        raise RuntimeError("No data found for robot {0}".format(agent))
    # fill the dataframe
    result = pd.DataFrame({col: data[col] for col in columns}, columns=columns)
    return result

