# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
import curses
import npyscreen


class MultiSlider(npyscreen.MultiLine):
    _contained_widgets = npyscreen.Slider

    def set_up_handlers(self):
        super().set_up_handlers()
        self.handlers.update(
            {
                curses.KEY_LEFT: self.h_decrease,
                curses.KEY_RIGHT: self.h_increase,
                ord("+"): self.h_increase,
                ord("-"): self.h_decrease,
            }
        )

    def _get_indexer(self):
        return self.cursor_line - self.start_display_at

    def _get_line(self):
        return self._my_widgets[self._get_indexer()]

    def h_increase(self, _input):
        line = self._get_line()
        line.h_increase(_input)
        self._handle_change(line)

    def h_decrease(self, _input):
        line = self._get_line()
        line.h_decrease(_input)
        self._handle_change(line)

    def _handle_change(self, line):
        line.update(clear=False)

        indexer = self._get_indexer()
        self.values[indexer] = line.get_value()

        # There appears to be a bug in npyscreen.MultiLine which causes the update check
        # to break. Therefore call this method explicitly on the changed widget
        line.when_check_value_changed()

    def display_value(self, vl):
        return float(vl)

    def set_is_line_cursor(self, line, value):
        line.editing = value


class MultiTitleSlider(MultiSlider):
    _contained_widgets = npyscreen.TitleSlider

    def __init__(self, *args, names=None, **keywords):
        self._names = names
        self._real_contained_widgets = self._contained_widgets
        super().__init__(*args, **keywords)

    def _get_line(self):
        return self._my_widgets[self._get_indexer()].entry_widget

    def set_is_line_cursor(self, line, value):
        super().set_is_line_cursor(line, value)
        line.entry_widget.editing = value

    def make_contained_named_widget(self, *args, **keywords):
        name = None

        if self._names:
            name = self._names[self._names_idx]
            self._names_idx = (self._names_idx + 1) % len(self._names)

        widget = self._real_contained_widgets(*args, name=name, **keywords)
        return widget

    def make_contained_widgets(self):
        self._names_idx = 0
        self._contained_widgets = self.make_contained_named_widget
        super().make_contained_widgets()


class MultiLineMultiTitleSlider(MultiTitleSlider):
    # The screen becomes too crowded if we were to only use 2 lines
    _contained_widget_height = 3

    def __init__(self, *args, names=None, **keywords):
        # Append characters to names until they are all of same length to ensure that
        # either all sliders are wrapped over two lines, or none are wrapped at all
        length_longest_name = len(max(names, key=len))
        names = [name.ljust(length_longest_name, ".") for name in names]

        keywords["use_two_lines"] = True
        super().__init__(*args, names=names, **keywords)
