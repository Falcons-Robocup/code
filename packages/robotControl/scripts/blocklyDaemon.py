""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 
import cherrypy
import multiprocessing
import os
import time
import signal
import falconspy

class Root(object):

    def __init__(self):
        self.executorProcesses = {}

    @cherrypy.expose
    def cmd(self, robotNr, pythoncode):

        scenarioStr = ""

        # Import WorldState
        scenarioStr += "import falconspy\n"
        scenarioStr += "import worldState\n"
        scenarioStr += "ws = worldState.WorldState(" + robotNr + ")\n"
        scenarioStr += "ws.startMonitoring()\n"

        # Import simScene for TeleportBall
        scenarioStr += "import simScene\n"

        # Import EnvironmentField
        scenarioStr += "import EnvironmentField\n"

        # Import FalconsCoordinates
        scenarioStr += "from FalconsCoordinates import *\n"

        # Make robotId available
        scenarioStr += "myRobotId = " + robotNr + "\n"

        # Connect to robot
        scenarioStr += "import robotControlInterface\n"
        scenarioStr += "rci = robotControlInterface.RobotControlInterface(" + robotNr + ")\n"
        scenarioStr += "rci.connect()\n"

        # The scenario
        scenarioStr += pythoncode

        # Shutdown
        scenarioStr += "rci.disconnect()\n"
        scenarioStr += "ws.stopMonitoring()\n"

        self.executorProcesses[robotNr] = multiprocessing.Process(target=self.execScenario, args=(scenarioStr,))
        self.executorProcesses[robotNr].daemon = True
        self.executorProcesses[robotNr].start()

        html_body = """
        Robot nr: %s\n
        Python:\n
        %s
        """ % (robotNr, scenarioStr)
        return html_body

    @cherrypy.expose
    def stopScenario(self, robotNr):
        if robotNr in self.executorProcesses.keys():
            os.kill(self.executorProcesses[robotNr].pid, signal.SIGKILL)

        # Reset match mode after terminating the scenario
        scenarioStr = "import robotControlInterface\n"
        scenarioStr += "rci = robotControlInterface.RobotControlInterface(" + robotNr + ")\n"
        scenarioStr += "rci.connect()\n"
        scenarioStr += "rci.disconnect()\n"
        exec(scenarioStr)

        return robotNr

    def execScenario(self, scenarioStr):
        exec(scenarioStr)

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

    s = input("Press any key to exit...")

    cherrypy.engine.exit()
