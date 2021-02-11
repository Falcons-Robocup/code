# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

import os

MAX_ROBOTS = 9

def guessAgentId():
    try:
        return int(os.getenv("TURTLE5K_ROBOTNUMBER"))
    except:
        return 0


