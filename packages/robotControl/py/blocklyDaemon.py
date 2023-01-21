# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0

import cherrypy
import dateutil.parser
import multiprocessing
import os
import time
import signal
import subprocess
import tempfile
import traceback
from cherrypy.lib.static import serve_file
from datetime import datetime

import falconspy
import robotLibrary
import kstplot
import plotdata
from plotfile import PlottableTextFile

class ScenarioAbort(Exception):
  """Custom exception class to signal an execution abort by the scenario"""


class Root(object):

    def __init__(self):
        self.executorProcesses = {}
        self.executorMessageQueue = {}
        self.manager = multiprocessing.Manager()

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def cmd(self, robotNr, pythoncode):

        runningScenario = self.executorProcesses.get(robotNr)

        # Client can perform a GET to block until all scenarios succeed
        if cherrypy.request.method == "GET":
            if runningScenario:
                runningScenario.join()
            return {
                "status": "done",
                "robotNr": robotNr
            }

        # Prevent running multiple scenarios at once
        if runningScenario:
            return {
                "status": "error",
                "error": {
                  "message": "Another scenario is already running on this robot",
                },
                "robotNr": robotNr
            }

        # Get the RDL file location on the robot
        newest_rdl_on_robot = subprocess.check_output("newestRDL").strip().decode()
        start = datetime.now()

        self.executorMessageQueue[robotNr] = self.manager.Queue()
        self.executorProcesses[robotNr] = multiprocessing.Process(target=self.execScenario, args=(robotNr, pythoncode))
        self.executorProcesses[robotNr].daemon = True
        self.executorProcesses[robotNr].start()

        # wait until MATCH_MODE is reached -- scenario ran to completion, or was stopped.
        self.executorProcesses[robotNr].join()
        del self.executorProcesses[robotNr]

        # Trigger the queue to be removed to prevent never-ending waits
        self.executorMessageQueue[robotNr].put(None)

        end = datetime.now()

        return {
            "status": "done",
            "robotNr": robotNr,
            "start": start.isoformat(),
            "end": end.isoformat(),
            "rdl_on_robot": newest_rdl_on_robot,
        }

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def stopScenario(self, robotNr):
        if robotNr in self.executorProcesses.keys():
            os.kill(self.executorProcesses[robotNr].pid, signal.SIGKILL)

        # Reset match mode after terminating the scenario
        scenarioStr = "import robotControlInterface\n"
        scenarioStr += "rci = robotControlInterface.RobotControlInterface(" + robotNr + ")\n"
        scenarioStr += "rci.connect()\n"
        scenarioStr += "rci.disconnect()\n"
        exec(scenarioStr)

        return {
            "status": "done",
            "robotNr": robotNr
        }

    def execScenario(self, robotNr, scenarioStr):
        # Make robotId available
        myRobotId = int(robotNr)

        # Instantiate RobotLibrary
        robotLib = robotLibrary.RobotLibrary(myRobotId)

        # Only execute scenario if the robot is not occupied with another scenario (or RobotCLI)
        if robotLib.isInMatchMode():

            # Connect to the robot
            robotLib.connect()

            try:
                robotLib.scenarioRunFromString(scenarioStr)
            except Exception as e:
                traceback.print_exc()
                self.add_message(robotNr, 'error', str(e), title=type(e).__name__)

            # Disconnect from the robot
            robotLib.disconnect()

        else:
            # If the robot was occupied, return an error
            self.add_message(myRobotId, 'error', "Error: Robot r%s is already in TestMode. Please wait for ongoing scenarios to finish, or restart the robot software." % (myRobotId), title="Robot occupied")


    def add_message(self, robotNr, severity, message, title=None):
        message = {
            'severity': severity,
            'message': message
        }

        if title:
            message['title'] = title

        self.executorMessageQueue[robotNr].put(message)


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def message(self, robotNr):
        # TODO: What about serving multiple clients at the same time?
        if robotNr not in self.executorMessageQueue:
            return {}

        message = self.executorMessageQueue[robotNr].get()

        if message is None:
            del self.executorMessageQueue[robotNr]
            return {}

        return message


    @cherrypy.expose
    def plotfile(self, robotNr, start, end, rdl_on_robot=None):

        # Get the RDL timestamp it was created on the robot
        time_rdl_creation = dateutil.parser.parse( " ".join( os.path.basename(rdl_on_robot).split(".")[0].split("_")[0:2] ) )

        # Determine "ageMin" for optimal parsing of the RDL
        time_before_action = datetime.fromisoformat(start)
        ageMin = (time_before_action - time_rdl_creation).seconds

        # Determine "ageMax" for optimal parsing of the RDL
        time_after_action = datetime.fromisoformat(end)
        ageMax = (time_after_action - time_rdl_creation).seconds

        # Add extra margin to ageMin / ageMax to capture all data
        ageMin -= 1
        ageMax += 1

        # Parse RDL into plottable textfile on robot
        _, tmp = tempfile.mkstemp(prefix='blockly-run-')
        textfile = PlottableTextFile(plotdata.PlotData.PLOT_ROBOT_VEL, int(robotNr), tmp)
        textfile.writeFileFromRDL(rdl_on_robot, ageMin, ageMax)

        # Convert the plottable textfile into a self-contained file to be opened by the KST Plot Viewer
        kstfile = kstplot.create(tmp, start=time_before_action)
        display_filename = 'blockly-run--' + time_before_action.strftime('%Y-%m-%d--%H-%M-%S') + '.kstplot'

        return serve_file(kstfile, disposition='attachment', content_type='octet-stream',name=display_filename)


if __name__ == '__main__':

    # Serve cherrypy
    conf = { 
            '/':
                {
                    'log.screen': False,
                    'log.access_file': '',
                    'log.error_file': '',
                    'tools.staticdir.on': True,
                    'tools.staticdir.dir': falconspy.FALCONS_CODE_PATH + '/packages/robotControl/blockly',
                    'tools.staticdir.index': 'index.html',
                },
            '/google-blockly': 
                {
                    'log.screen': False,
                    'log.access_file': '',
                    'log.error_file': '',
                    'tools.staticdir.on': True,
                    'tools.staticdir.dir': falconspy.FALCONS_DATA_PATH + '/external/google-blockly',
                },
            '/jquery': 
                {
                    'log.screen': False,
                    'log.access_file': '',
                    'log.error_file': '',
                    'tools.staticdir.on': True,
                    'tools.staticdir.dir': falconspy.FALCONS_DATA_PATH + '/external/jquery',
                },
            }
    cherrypy.config.update({
                            'log.screen': False,
                            'log.access_file': '',
                            'log.error_file': '',
                            })
    cherrypy.log.error_log.propagate = False
    cherrypy.log.access_log.propagate = False
    cherrypy.tree.mount(Root(), '/', conf)
    cherrypy.server.socket_host = '0.0.0.0'
    cherrypy.engine.start()
    cherrypy.engine.block()
