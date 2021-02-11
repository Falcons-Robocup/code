# Copyright 2019 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python


import sys, argparse
import matplotlib.pyplot as plt



def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot interpolation data- and curve")
    parser.add_argument("-s", "--samples", type=str, default="/var/tmp/interpolationSampleData.txt", help="file containing sample data")
    parser.add_argument("-c", "--curve", type=str, default="/var/tmp/interpolationCurve.txt", help="file containing interpolated curve")
    return parser.parse_args()


def loadFile(filename):
    f = open(filename, 'r')
    lines = f.readlines()
    x = []
    y = []
    for line in lines:
        v = line.split()
        if len(v) == 2:
            x.append(float(v[0]))
            y.append(float(v[1]))
    return (x, y)


def run(fileSamples, fileCurve):
    (xs, ys) = loadFile(fileSamples)
    (xc, yc) = loadFile(fileCurve)
    plt.figure()
    plt.plot(xs, ys, 'rx', label='samples')
    plt.plot(xc, yc, 'b:', label='curve')
    plt.show()


if __name__ == '__main__':
    args = parse_arguments()
    run(args.samples, args.curve)

