# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
import argparse


class ExplicitDefaultsHelpFormatter(argparse.ArgumentDefaultsHelpFormatter):
    """Only show default values for arguments which explicitly have a default configured"""

    def _get_help_string(self, action):
        if action.default in (None, False):
            return action.help
        return super()._get_help_string(action)
