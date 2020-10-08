""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import time
import threading

import sys
import falconspy
import rtdb2
import sharedTypes

class RobotControl():

    def __init__(self, robotId):
        self._robotId = robotId
        assert(self._robotId in range(1, 20))

        # So far unused
        ## Simulated?
        #hostname = os.uname()[1]
        #self.isSimulated = bool("FALCON" or "TURTLE" in hostname)

        # Initialize stimulation to None
        self.clearStimulation()

        # Initialize stimulatorThread
        # daemon=True means thread is killed during shutdown of main thread, and not block main thread from shutdown
        self._stimulatorThread = threading.Thread(target=self._threadExecutor)
        self._stimulatorThread.setDaemon(True)
        self._stimulatorThread.start()


    ####################
    # Public functions #
    ####################

    def connect(self):

        # Initialize RTDB connection
        self._rtdb2Store = rtdb2.RtDB2Store(rtdb2.RTDB2_DEFAULT_PATH, readonly=False) # write mode
        self._rtdb2Store.refresh_rtdb_instances()

        # Initialize stimulation to None
        self.clearStimulation()

        # Set robot in TestMode
        self._setTestMode() # tell teamplay to not listen to heartbeat anymore, giving control to this library

        # Store BH state -- used to restore BH state when disconnecting
        self._bhState = self._rtdbGet("BALLHANDLERS_SETPOINT", timeout=None)

        # Enable BallHandlers on startup
        time.sleep(0.1) # SW still running, wait for a bit before enabling BH
        self._rtdbPut("BALLHANDLERS_SETPOINT", True)

    def disconnect(self):

        # Reinitialize stimulation to None
        self.clearStimulation()

        # Restore BH state
        self._rtdbPut("BALLHANDLERS_SETPOINT", self._bhState.value)

        # Restore robot to MatchMode
        self._setMatchMode()

        # Close RTDB connection
        self._rtdb2Store.closeAll()

    def stimulate(self, rtdb_entries):
        self._rtdb_entries = rtdb_entries
        self._stimulateOnce = False

        # Add a small sleep to allow this function to work in a "while True"
        time.sleep(1.0 / 30.0) 

    def stimulateOnce(self, rtdb_entries):
        self._rtdb_entries = rtdb_entries
        self._stimulateOnce = True

        # Add a small sleep to allow this function to work in a "while True"
        time.sleep(1.0 / 30.0) 

    def clearStimulation(self):
        self._rtdb_entries = None
        self._stimulateOnce = False

    def blockUntilTPOverridePassedOrFailed(self):

        # First write INVALID to avoid the race condition that the result from the previous action/behavior is used.
        self._rtdbPut("TP_OVERRIDE_RESULT", {'status': sharedTypes.behTreeReturnEnum.INVALID.value} )

        # Keep looping until PASSED or FAILED is found
        while True:
            tpOverrideResult = self._rtdbGet("TP_OVERRIDE_RESULT")
            if tpOverrideResult.value["status"] == sharedTypes.behTreeReturnEnum.PASSED.value or tpOverrideResult.value["status"] == sharedTypes.behTreeReturnEnum.FAILED.value:
                break
            else:
                time.sleep(1.0 / 30.0) 

    def sleep(self, duration):
        time.sleep(duration)

    #####################
    # Private functions #
    #####################

    # This thread is always running
    def _threadExecutor(self):

        while True:

            # If a key and value is known, put the data in RTDB as part of the execution architecture
            if self._rtdb_entries is not None:
                for rtdb_entry in self._rtdb_entries:
                    self._rtdbPut(rtdb_entry[0], rtdb_entry[1])

            # If the stimulation only needs to happen once, clear stimulation now
            if self._stimulateOnce:
                self.clearStimulation()

            # Sleep to maintain 30Hz
            time.sleep(1.0 / 30.0) 


    def _rtdbGet(self, key, *args, **kwargs):
        return self._rtdb2Store.get(self._robotId, key, *args, **kwargs)

    def _rtdbPut(self, key, value):
        return self._rtdb2Store.put(self._robotId, key, value)

    def _setMatchMode(self):
        self._rtdbPut("MATCH_MODE", True)

    def _setTestMode(self):
        self._rtdbPut("MATCH_MODE", False)
