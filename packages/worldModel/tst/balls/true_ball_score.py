# Copyright 2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import sys

import falconspy
import rdlLib

import math

def calculate_rdl_score(rdlfile, agent, debug_print=False):
    error_sqr = 0.0
    num_measusrements = 0

    rdl = rdlLib.RDLFile(rdlfile)
    rdl.openRDL()
    while True:
        frame = rdl.readNextFrame()
        if frame is None:
            break

        agentData = frame.data[agent]             

        if 'BALLS' in agentData:
            wm_balls = agentData['BALLS']

            if len(wm_balls.value):
                wm_ball_pos = wm_balls.value[0][0]  
                
                if ('BALLHANDLERS_BALL_POSSESSION' in agentData) and agentData['BALLHANDLERS_BALL_POSSESSION'].value:
                    continue

                if 'DIAG_TRUE_BALL' in agentData:
                    true_ball = agentData['DIAG_TRUE_BALL']
                    true_ball_pos = [true_ball.value['x'], true_ball.value['y'], true_ball.value['z']]

                    for (v1, v2) in zip(wm_ball_pos, true_ball_pos):
                        if (not math.isnan(v2)):
                            derror = (v1 - v2)**2
                            derror = min(1.0, derror)

                            if math.isnan(derror):
                                derror = 1.0

                            error_sqr += derror
                            num_measusrements += 1

                            if debug_print:
                                print(derror, wm_ball_pos, true_ball_pos)

    return (error_sqr, num_measusrements)


if __name__ == "__main__":
    rdlfile = sys.argv[1]
    agent = int(sys.argv[2])

    debug_print = False
    if (len(sys.argv) > 3):
        debug_print = sys.argv[3] == 'p'

    (error_sqr, num_measusrements) = calculate_rdl_score(rdlfile, agent, debug_print)

    print('Total Error: %f'%(error_sqr)) 
    print('Measurements: %d'%(num_measusrements)) 
    print('Average Error: %f'%(error_sqr/num_measusrements)) 
