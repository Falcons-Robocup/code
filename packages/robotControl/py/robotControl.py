# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import time
import threading

import sys
import falconspy
import falconsrtdb
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

        # Initialize frequency to 40Hz
        self._frequency = 40.0

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
        self._rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=False) # write mode
        self._rtdbStore.refresh_rtdb_instances()

        # Initialize stimulation to None
        self.clearStimulation()

        # Set robot in TestMode
        self._setTestMode() # tell teamplay to not listen to heartbeat anymore, giving control to this library

        # Store frequency from execution
        config_execution = self._rtdbGet("CONFIG_EXECUTION", timeout=None)
        self._frequency = config_execution.value["frequency"]

        # Store BH state -- used to restore BH state when disconnecting
        self._bhState = self._rtdbGet("BALLHANDLERS_SETPOINT", timeout=None)

        # Enable BallHandlers on startup
        time.sleep(0.1) # SW still running, wait for a bit before enabling BH
        self._rtdbPut("BALLHANDLERS_SETPOINT", True)

    def disconnect(self):

        # Reinitialize stimulation to None
        self.clearStimulation()

        # Restore BH state (if available)
        if self._bhState is not None:
            self._rtdbPut("BALLHANDLERS_SETPOINT", self._bhState.value)

        # Restore robot to MatchMode
        self._setMatchMode()

        # Close RTDB connection
        self._rtdbStore.closeAll()

    def stimulate(self, rtdb_entries):
        self._rtdb_entries = rtdb_entries

        # Add a small sleep to allow this function to work in a "while True"
        self._sleepOneTick()

    def stimulateOnce(self, rtdb_entries, do_sleep=True):
        for rtdb_entry in rtdb_entries:
            self._rtdbPut(rtdb_entry[0], rtdb_entry[1])

        if do_sleep:
            # Add a small sleep to allow this function to work in a "while True"
            self._sleepOneTick()

    def clearStimulation(self):
        self._rtdb_entries = None

    def blockUntilTPOverridePassedOrFailed(self):

        # First write INVALID to avoid the race condition that the result from the previous action/behavior is used.
        self._rtdbPut("TP_OVERRIDE_RESULT", {'status': sharedTypes.behTreeReturnEnum.INVALID.value} )

        # Keep looping until PASSED or FAILED is found
        while True:
            tpOverrideResult = self._rtdbGet("TP_OVERRIDE_RESULT")
            if tpOverrideResult.value["status"] == sharedTypes.behTreeReturnEnum.PASSED.value or tpOverrideResult.value["status"] == sharedTypes.behTreeReturnEnum.FAILED.value:
                break
            else:
                self._sleepOneTick()

    def blockUntilMPPassedOrFailed(self):

        # First write INVALID to avoid the race condition that the result from the previous action/behavior is used.
        self._rtdbPut("ACTION_RESULT", [sharedTypes.actionResultTypeEnum.INVALID.value] )

        # Keep looping until PASSED or FAILED is found
        while True:
            mpResult = self._rtdbGet("ACTION_RESULT")
            if mpResult.value[0] == sharedTypes.actionResultTypeEnum.PASSED.value:
                return True
            elif mpResult.value[0] == sharedTypes.actionResultTypeEnum.FAILED.value:
                return False
            else:
                self._sleepOneTick()

    def blockUntilVelocitySettled(self, settleTime):

        settling = False
        settlingStartTime = None
        EPSILON = 1E-4

        # Keep looping until the robot velocity remains 0.0 for at least `settleTime` seconds
        while True:

            robotVel = self._rtdbGet("ROBOT_VELOCITY_SETPOINT")

            # if  (robotVel.x < EPSILON)  &&  (robotVel.y < EPSILON)  &&  (robotVel.Rz < EPSILON)
            if abs(robotVel.value[0]) < EPSILON and abs(robotVel.value[1]) < EPSILON and abs(robotVel.value[2]) < EPSILON:

                if not settling:
                    # start settling
                    settling = True
                    settlingStartTime = time.perf_counter()

                else:
                    # if we have been settling for `settleTime` seconds, we are done
                    if (time.perf_counter() - settlingStartTime) > settleTime:
                        break
            else:
                # new robot velocity, reset settling state
                settling = False
                settlingStartTime = None

            self._sleepOneTick()


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
                self.stimulateOnce(self._rtdb_entries, do_sleep=False)

            # Sleep to maintain frequency
            self._sleepOneTick()


    def _rtdbGet(self, key, *args, **kwargs):
        return self._rtdbStore.get(self._robotId, key, *args, **kwargs)

    def _rtdbPut(self, key, value):
        return self._rtdbStore.put(self._robotId, key, value)

    def _setMatchMode(self):
        self._rtdbPut("MATCH_MODE", True)

    def _setTestMode(self):
        self._rtdbPut("MATCH_MODE", False)

    def _sleepOneTick(self):
        time.sleep(1.0 / self._frequency) 
