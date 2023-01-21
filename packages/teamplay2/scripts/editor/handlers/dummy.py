# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
import json
from random import random

from .base import BaseHandler


class DummyHandler(BaseHandler):
    def load(self):
        heightmaps = [
            "CUSTOM",
            "DEFEND_ATTACKING_OPPONENT",
            "MOVE_TO_FREE_SPOT",
            "DRIBBLE",
            "POSITION_FOR_OPP_SETPIECE",
            "POSITION_ATTACKER_FOR_OWN_SETPIECE",
        ]

        factors = [
            "AVOID_BALL",
            "AVOID_OBSTACLES",
            "AVOID_TEAM_MATES",
            "BETWEEN_POI_AND_CLOSEST_OBSTACLE",
            "CLOSE_TO_BALL_CLAIMED_LOCATION",
            "CLOSE_TO_OWN_POS",
            "IN_FRONT_OF_OPP_GOAL",
            "NEAR_OBSTACLES",
            "NEAR_OWN_GOAL",
            "OBS_BLOCKING_BALL",
            "OBS_BLOCKING_OPP_GOAL",
            "OBS_BLOCKING_TEAMMATES",
        ]

        self.data = {
            "heightmaps": {
                "factors": {
                    heightmap: {factor: random() for factor in factors}
                    for heightmap in heightmaps
                }
            }
        }

    def save(self):
        with open("factors.json", "w") as outfile:
            outfile.write(json.dumps(self.data, indent=2))
