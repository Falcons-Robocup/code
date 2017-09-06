#!/usr/bin/env python
#
# JFEI 2016-05-03 
# Plot inputs and Kalman results together.

 
import sys, os
import matplotlib.pyplot as plt



# data as map with time as key
vision = {}
vision["t"] = []
vision["x"] = []
vision["y"] = []
vision["phi"] = []
kalman = {}
kalman["t"] = []
kalman["x"] = []
kalman["y"] = []
kalman["phi"] = []


# parse input files
for f in sys.argv[1:]:
    for line in file(f).readlines():
        words = line.split()
        if (len(words) == 5) and (words[1] in ["kalman", "vision"]):
            t = float(words[0])
            x = float(words[2])
            y = float(words[3])
            phi = float(words[4])
            d = kalman
            if words[1] == "vision":
                d = vision
            d["t"].append(t)
            d["x"].append(x)
            d["y"].append(y)
            d["phi"].append(phi)


# plot the result
plt.plot(vision["t"], vision["x"], 'rx', kalman["t"], kalman["x"], 'ro', vision["t"], vision["y"], 'bx', kalman["t"], kalman["y"], 'bo', vision["t"], vision["phi"], 'gx', kalman["t"], kalman["phi"], 'go')
#plt.plot(vision["t"], vision["x"], 'rx', kalman["t"], kalman["x"], 'ro', vision["t"], vision["y"], 'bx', kalman["t"], kalman["y"], 'bo')
plt.show()
