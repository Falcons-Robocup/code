# Copyright 2022 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3 -u


# python/system libraries
import os
import argparse

# Falcons libraries
import falconspy
import rtdb2tools
import analyze_lib as lib
import dataframe



def parse_arguments():
    descriptionTxt = """Dump data from RDL in a txt file."""
    parser     = argparse.ArgumentParser(description=descriptionTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-f', '--frequency', help='resample frequency to use (typically best to slightly undersample heartbeat)', type=float)
    parser.add_argument('-o', '--output', help='output filename', type=str, default='/var/tmp/dataframe_<rdlfile>.<ext>')
    parser.add_argument('-c', '--csv', help='write a csv file', action='store_true')
    parser.add_argument('-q', '--quiet', help='suppress output', action='store_true')
    # TODO: options to filter time range?
    parser.add_argument('rdlfile', help='analyze given RDL file', type=str)
    return parser.parse_args()



if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    # guess robot from RDL file name, choose suitable data sampling frequency
    agent = args.robot
    frequency = args.frequency
    if agent == 0:
        agent = lib.get_agent_from_rdl_filename(args.rdlfile)
        if frequency == None:
            frequency = 25.0 # subsample robot 30Hz heartbeat
    else:
        if frequency == None:
            frequency = 15.0
    # run
    df = dataframe.DataFrame(args.rdlfile, agent, verbose=not args.quiet)
    extension = 'txt'
    if args.csv:    
        extension = 'csv'
    outputfilename = args.output.replace('<rdlfile>', os.path.basename(args.rdlfile)).replace('<ext>', extension)
    if args.csv:    
        df.writeAsCSV(outputfilename)
    else:
        df.writeAsTXT(outputfilename)


