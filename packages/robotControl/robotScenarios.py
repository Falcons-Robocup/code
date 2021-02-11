# Copyright 2016-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Python library for advanced sequences and demo's, on-robot.
#
# Implementations:
#    robotCLI.py          # command-line interface
#    robotLibrary.py      # discloses all basic commands and scenarios, also includes parser
#    robotScenarios.py    # scenario interface
#    robotInterface.py # basic commands and their ROS interfaces
#    scenarios/*.py       # scenario implementations
#
# Jan Feitsma, 2016-12-17


import inspect


# load all scenarios dynamically from the folder 'scenarios', which has a special __init.py__
import scenarios



def listScenarios():
    """
    Return the dynamically loaded list of scenario functions (as strings).
    Use getScenario to get the function object.
    """
    result = []
    for f in scenarios.__all__:
        result += [m[0] for m in inspect.getmembers(getattr(scenarios, f), inspect.isfunction) if "scenarios." in m[1].__module__ and m[0][0] != '_']
    return result


def getScenario(name):
    """
    Return the function which belongs to given name, for example 'driveCircle' or 'intercept'.
    """
    if not name in listScenarios():
        raise Exception("unrecognized scenario '%s'" % (name))
    scenarioFunction = None
    for f in scenarios.__all__:
        if hasattr(getattr(scenarios, f), name):
            return getattr(getattr(scenarios, f), name)
    raise Exception("internal error")


def runScenario(name, *args):
    """
    Execute given scenario with optional arguments.
    If called from command line, then arguments will all be strings. Scenario functions must sanitize inputs.
    """
    # check the dynamically loaded list of scenarios
    scenarioFunction = getScenario(name)
    # call it
    scenarioFunction(*args)





