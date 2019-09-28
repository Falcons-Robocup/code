""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
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





