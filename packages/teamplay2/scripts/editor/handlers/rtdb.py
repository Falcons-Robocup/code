# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH
from .base import BaseHandler


class RtDBDataHandler(BaseHandler):
    def __init__(self, path, agent, key):
        super().__init__()
        self.agent = agent
        self.key = key
        self.rtdb2_store = RtDB2Store(path, False)  # don't start in read-only

    def load(self):
        entry = self.rtdb2_store.get(self.agent, self.key, timeout=None)
        assert entry is not None
        self.original_data = self.data = entry.value

    def save(self):
        parent_keys = self.parents.copy()
        key = parent_keys.pop()

        # Update original data with the selected content
        current_entry = self.original_data
        for parent in parent_keys:
            current_entry = current_entry[parent]

        current_entry[key] = self.data

        self.rtdb2_store.put(self.agent, self.key, self.original_data)
