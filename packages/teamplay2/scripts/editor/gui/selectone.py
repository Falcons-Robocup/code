# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
import npyscreen


class SelectOneAction(npyscreen.MultiSelectAction):
    _contained_widgets = npyscreen.RoundCheckBox

    def update(self, clear=True):
        # Always select the line which is currently highlighted
        self.value = [
            self.cursor_line,
        ]

        return super().update(clear)

    def actionHighlighted(self, act_on_this, key_press):
        # Default action is to exit the form
        self.parent.editing = False
        self.h_exit(key_press)


class TitleSelectOneAction(npyscreen.TitleMultiLine):
    _entry_type = SelectOneAction
