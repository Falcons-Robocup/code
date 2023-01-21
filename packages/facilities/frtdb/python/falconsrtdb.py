# Copyright 2021 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0

import os
import rtdb2
import FalconsEnv

class FalconsRtDBStore(rtdb2.RtDB2Store):
    """
    A FalconsRtDBStore contains all RtDB connections, one for each agent.
    Automatically chooses the correct agent:
    - 0 in simulation
    - [1-9] when on a robot
    """
    def __init__(self, readonly=True):

        # Determine agentId
        agentId = 0
        if not FalconsEnv.get_simulated():
            agentId = FalconsEnv.get_robot_num()

        # Build path:
        # RTDB2_DEFAULT_PATH/[0-9]/default/
        path = os.path.join( rtdb2.RTDB2_DEFAULT_PATH, str(agentId), "default" )

        # Init rtdb2.RtDB2Store
        super().__init__(path, readonly)
