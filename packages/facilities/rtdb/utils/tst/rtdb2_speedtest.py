# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#! /usr/bin/env python3

import falconspy

from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH

ITERATIONS = 10000

if __name__ == '__main__':
    print "Sending {} samples to RtDB...".format(ITERATIONS)

    rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False)
    rtdb2Store.refresh_rtdb_instances()

    ages = list()
    for i in range(ITERATIONS):
        rtdb2Store.rtdb_instances["agent_shared0"].put("TEST", i)
        (value, age) = rtdb2Store.rtdb_instances["agent_shared0"].get("TEST")
        ages.append(age)

    print "Got the following ages (lifes) :"
    print "Min: {} [ms]".format(min(ages))
    print "Max: {} [ms]".format(max(ages))
    print "Avg: {} [ms]".format(sum(ages)/len(ages))
