# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
import copy
import re
import yaml
from pathlib import Path

from .base import BaseHandler


class TeamplayConfigHandler(BaseHandler):
    def __init__(self, path):
        super().__init__()
        self.path = Path(path)

    def load(self):
        with self.path.open() as infile:
            self.data = yaml.load(infile.read(), Loader=yaml.FullLoader)

    def save(self):
        lines = self.path.read_text().splitlines()

        # Hacky way to determine whether we're dealing with a nested dict; check whether
        # the first item holds another dict.
        # As you can see, we currently only support one level of nested dicts
        if isinstance(next(iter(self.data.values())), dict):
            for key, value in self.data.items():
                lines = self._edit_yaml_inline(lines, self.parents + [key], value)
        else:
            lines = self._edit_yaml_inline(lines, self.parents.copy(), self.data)

        with self.path.open("w") as outfile:
            outfile.write("\n".join(lines))

    def _edit_yaml_inline(self, lines, parent_keys, data):
        output_lines = []
        output_data = copy.deepcopy(data)

        # This might seem like a rather complicated way of writing YAML content,
        # but we want to preserve the original formatting as much as possible
        for line in lines:
            # Exhaust parent keys until we have found all of them
            # e.g.:
            #   DRIBBLE:
            if parent_keys:
                key = parent_keys[0]
                if re.match(f"^\s*{key}\s*:\s*$", line):
                    parent_keys.pop(0)
            # Then exhaust every key/value pair, replace match with the updated value
            # e.g.:
            #   AVOID_OBSTACLES : 0.8
            else:
                for key in list(output_data.keys()):
                    match = re.match(f"^\s*{key}\s*:\s*([0-9.]+)\s*$", line)
                    if match:
                        value = "{:2.2f}".format(output_data.pop(key))
                        line = line.replace(match.group(1), value)
                        break

            output_lines.append(line)
        return output_lines
