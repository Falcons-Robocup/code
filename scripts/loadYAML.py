#!/usr/bin/env python
#


import sys
import argparse
import yaml

sys.path.append("/home/robocup/falcons/code/packages/facilities/rtdb3/src/tools/rtdb2Tools")
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH


def run(args):
    f = open(args.yamlfile, mode='r')
    y = yaml.load(f.read())
    f.close()

    # Create instance of RtDB2Store and read databases from disk
    rtdb2Store = RtDB2Store(args.path, False) # don't start in read-only

    # Get current value
    value = rtdb2Store.get(args.agent, args.key)

    # Assign yaml values
    value = y

    # Store and finish
    rtdb2Store.put(args.agent, args.key, value)
    rtdb2Store.closeAll()


def guessAgentId():
    try:
        return int(os.getenv("TURTLE5K_ROBOTNUMBER"))
    except:
        return 0


if __name__ == '__main__':
    # Argument parsing.
    descriptionTxt = 'Load configuration yaml file into given RTDB key.\n'
    exampleTxt = 'Example: loadYaml.py -k CONFIG_PATHPLANNING ~/falcons/code/config/PathPlanningSim.yaml\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-k', '--key', help='RTDB key to write to', required=True)
    parser.add_argument('-a', '--agent', help='agent ID to use, default guess', type=int, default=guessAgentId())
    parser.add_argument('-p', '--path', help='database path to use', type=str, default=RTDB2_DEFAULT_PATH)
    parser.add_argument('yamlfile', help='yaml file to load')
    args       = parser.parse_args()

    # run
    run(args)


