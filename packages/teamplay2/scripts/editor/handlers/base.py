# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
class BaseHandler:
    def __init__(self):
        self.data = {}
        self.parents = []

    def load(self):
        raise NotImplementedError

    def select(self, key):
        self.parents.append(key)
        self.data = self.data[key]

    def keys(self):
        return list(self.data.keys())

    def values(self):
        return list(self.data.values())

    def update(self, key, value):
        self.data[key] = value
        self.save()

    def save(self):
        raise NotImplementedError
