# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
import os
import argparse
from collections import OrderedDict
import numpy as np
from rdlLib import RDLFile



def analyze(timestamps):
    prev_k = None
    t0 = None
    for (k, v) in timestamps.items():
        if t0 == None:
            t0 = v[0]
        a = np.array(v)
        deltas = (a - np.roll(a, 1))[1:]
        print('key: ' + k)
        print('   count        : {:5d}'.format(len(a)))
        print('   frequency    : {:9.3f} Hz'.format(1.0 / np.mean(deltas)))
        print('   jitter       : {:9.3f} ms (1 stddev)'.format(1e3 * np.std(deltas)))
        if prev_k:
            print('   duration from {:s} to {:s}:'.format(prev_k, k))
            # for each timestamp, find matching timestamp
            durations = []
            for t in a:
                d = t - prev_a
                duration = min(d[d>=0])
                durations.append(duration)
            print('      count     : {:5d}'.format(len(durations)))
            print('      mean      : {:9.3f} ms'.format(1e3 * np.mean(durations)))
            print('      stddev    : {:9.3f} ms'.format(1e3 * np.std(durations)))
            print('      max       : {:9.3f} ms'.format(1e3 * max(durations)))
        prev_k = k
        prev_a = a


def run(rdlfile, agent, analyze_keys):
    # prepare timestamp lists
    timestamps = OrderedDict()
    for key in analyze_keys:
        timestamps[key] = []

    # go through the frames
    rdlFile = RDLFile(args.file)
    rdlFile.parseRDL()
    frameCounter = 0
    for frame in rdlFile.frames:
        if agent in frame.data.keys():
            for key in frame.data[agent].keys():
                if key in analyze_keys:
                    item = frame.data[agent][key]
                    t = item.timestamp[0] + 1e-6 * item.timestamp[1]
                    timestamps[key].append(t)
        # next
        frameCounter += 1

    # check for missing keys
    for key in analyze_keys:
        if key not in timestamps.keys():
            raise Exception('no data found for agent {}, key {}'.format(agent, key))
        if len(timestamps[key]) == 0:
            raise Exception('no data found for agent {}, key {}'.format(agent, key))

    # analyze
    analyze(timestamps)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze latency and jitter from RDL.')
    parser.add_argument('-a', '--agent', type=int, help='which agent to analyze')
    parser.add_argument('file', help='path to RDL file')
    parser.add_argument('key', type=str, nargs='+', help='which key sequence to analyze')
    args = parser.parse_args()

    # check arguments
    if not os.path.exists(args.file):
        print('ERROR: file \'{}\' not found.'.format(args.file))
        exit(1)
    if len(args.key) == 0:
        print('ERROR: require one or more keys to be specified')
        exit(1)
    if args.agent == None:
        print('ERROR: require agent id')
        exit(1)

    # run
    run(args.file, args.agent, args.key)

