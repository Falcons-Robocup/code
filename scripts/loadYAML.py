#!/usr/bin/env python3
#


import sys
import argparse
import yaml

import falconspy
import sharedTypes
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH



def enumConverter(v):
    # Hook for SmartAssignment, to solve enum string2int conversion (#83)
    # Without this hook, users should specify raw enum integer values in yaml, which is not good for usability and readibility
    # With this hook, users instead can specify the enum value as string, BUT must prefix the enum type
    # This hook can convert it into the associated integer value, for use in RTDB / serialization
    # See for example PathPlanning.yaml:
    #    velocityControllerType:     velocitySetpointControllerTypeEnum:PID

    # check if v is a string and has the "enum:" prefix, if not, do nothing and return
    if not isinstance(v, str):
        return (False, v)
    parts = v.split(":")
    if len(parts) == 1:
        return (False, v)

    # check for valid enum type
    enumTypeName = parts[0]
    try:
        enumType = getattr(sharedTypes, enumTypeName)
    except:
        raise Exception("enum type string {} not known to sharedTypes".format(enumTypeName))
    enumStrValue = parts[1]
    try:
        enumIntValue = getattr(enumType, enumStrValue).value
    except:
        raise Exception("enum value string {} not known to sharedTypes.{}".format(enumStrValue, enumTypeName))

    # successful conversion
    v = enumIntValue
    return (True, v)


class SmartAssignment():
    """
    Smart dict assignment.
    Will check for type compatibility and completeness.
    """

    def __init__(self, target, source, hook=None, targetName="target", sourceName="source", strict=False):
        self.strict = strict
        self.hook = hook
        self.targetName = targetName
        self.sourceName = sourceName
        self.assign(target, source)

    def assign(self, target, source, stack = []):

        # check input types
        locStr = ""
        if len(stack):
            locStr = " at " + ".".join(stack)
        if not isinstance(target, dict):
            raise Exception("expecting a dict for {}{}, got {} with value '{}'".format(self.targetName, locStr, type(target), str(target)))
        if not isinstance(source, dict):
            raise Exception("expecting a dict for {}{}, got {} with value '{}'".format(self.sourceName, locStr, type(source), str(source)))

        # check key set equality: target.keys() - source.keys() 
        keyDiff = set(target.keys()).difference(set(source.keys()))
        if len(keyDiff) > 0:
            if self.strict:
                raise Exception("missing key {} in {} dict".format(".".join(stack + [keyDiff.pop()]), self.sourceName))
            else:
                print("WARNING: missing key {} in {} dict".format(".".join(stack + [keyDiff.pop()]), self.sourceName))

        # check key set equality: source.keys() - target.keys()
        keyDiff = set(source.keys()).difference(set(target.keys()))
        if len(keyDiff) > 0:
            if self.strict:
                raise Exception("missing key {} in {} dict".format(".".join(stack + [keyDiff.pop()]), self.targetName))
            else:
                print("WARNING: missing key {} in {} dict".format(".".join(stack + [keyDiff.pop()]), self.targetName))

        # go through the keys, copying all source keys towards the target
        for k in source.keys():
            done = False
            # run the hook
            if self.hook != None:
                (done, v) = self.hook(source[k])
            if not done:
                # implicit cast, to e.g. promote int to float
                if k in target:
                    try:
                        t = type(target[k])
                        v = t(source[k])
                    except:
                        raise Exception("type mismatch for key '{}': got {} with value '{}', expected {}, ".format(".".join(stack + [k]), type(source[k]), str(source[k]), type(target[k])))
                else:
                    v = source[k]
            # recurse?
            if k in target and isinstance(target[k], dict):
                self.assign(target[k], v, stack + [k])
            else:
                target[k] = v


def run(args):

    # Load the yaml
    f = open(args.yamlfile, mode='r')
    y = yaml.load(f.read(), Loader=yaml.FullLoader)
    f.close()

    # Create instance of RtDB2Store and read databases from disk
    rtdb2Store = RtDB2Store(args.path, False) # don't start in read-only

    # Get current value
    # It is typically only written once when a process initializes, so do not use timeout flag
    item = rtdb2Store.get(args.agent, args.key, timeout=None)
    if item is None:
        # It is possible the key does not exist yet.
        # So write the the entire YAML to RtDB.
        print("WARNING: Failed to get key='{}' with agent='{}' from RtDB. Assuming key does not yet exist, and writing new values to RtDB.".format(args.key, args.agent))

        itemvalue = {}
        SmartAssignment(itemvalue, y, hook=enumConverter, targetName="rtdb", sourceName="yaml", strict=False)

        # Store key in RtDB
        rtdb2Store.put(args.agent, args.key, itemvalue)

    else:
        # Assign yaml values, like
        #     item.value = y
        # but with more features:
        # * type checks
        # * completeness checks
        # * enum/string conversions
        SmartAssignment(item.value, y, hook=enumConverter, targetName="rtdb", sourceName="yaml", strict=(not args.nostrict))

        # Store key back to RtDB
        rtdb2Store.put(args.agent, args.key, item.value)

    # cleanup
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
    parser.add_argument('-s', '--nostrict', help='disable strict type/key checking', action='store_true')
    parser.add_argument('yamlfile', help='yaml file to load')
    args       = parser.parse_args()

    # run
    run(args)


