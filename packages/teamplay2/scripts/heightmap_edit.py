# Copyright 2022 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3
import argparse
from enum import Enum
import os
from pathlib import Path
import sys

import npyscreen

import falconspy
import rtdb2tools
from rtdb2 import RTDB2_DEFAULT_PATH

from editor.args.formatter import ExplicitDefaultsHelpFormatter
from editor.gui.multislider import MultiTitleSlider
from editor.gui.selectone import TitleSelectOneAction
from editor.handlers.config import TeamplayConfigHandler
from editor.handlers.dummy import DummyHandler
from editor.handlers.rtdb import RtDBDataHandler


class HeightmapFactorSlider(npyscreen.Slider):
    def __init__(self, screen, **keywords):
        super().__init__(screen, step=0.05, out_of=1.000, **keywords)

    def translate_value(self):
        return "{:2.2f}".format(self.value)


class BoxedHeightmapFactorSlider(npyscreen.BoxTitle):
    _contained_widget = HeightmapFactorSlider


class HeightmapFactorMultiTitleSliderSlider(MultiTitleSlider):
    _contained_widgets = BoxedHeightmapFactorSlider
    _contained_widget_height = 3

    def set_handler(self, handler):
        self.handler = handler

    def make_contained_named_widget(self, *args, **keywords):
        keywords['value_changed_callback'] = self.widget_changed_callback
        return super().make_contained_named_widget(*args, **keywords)

    def widget_changed_callback(self, widget):
        self.handler.update(widget.name, widget.value)


class HeightmapEditorApp(npyscreen.NPSApp):
    def __init__(self):
        self.heightmap = None
        self.handler = None

    def main(self):

        # Have a heightmap selected if none provided
        # TODO: Make form quit on pressing 'q'
        if not self.heightmap:
            F = npyscreen.Form(name="Heightmap Editor")
            title = "Select a heightmap:"
            widget = F.add(
                TitleSelectOneAction,
                name=title,
                begin_entry_at=len(title) + 3,
                values=self.handler.keys(),
            )

            F.edit()
            self.heightmap = widget.get_selected_objects()[0]

        # Edit the provided heightmap
        # TODO: Make form quit on pressing 'q'
        self.handler.select(self.heightmap)

        F = npyscreen.Form(name="Heightmap Editor -- " + self.heightmap)
        widget = F.add(
            HeightmapFactorMultiTitleSliderSlider,
            names=self.handler.keys(),
            values=self.handler.values(),
            allow_filtering=False,
        )

        widget.set_handler(self.handler)
        F.edit()


class Source(Enum):
    CONFIG = "config"
    DUMMY = "dummy"
    RTDB = "rtdb"

    def __str__(self):
        return self.value


if __name__ == "__main__":

    # Argument parsing.
    descriptionTxt = "This tool allows you to edit an RtDB item in the database given an RtDB key.\n"
    exampleTxt = "Example: rtdb2_edit.py CONFIG_PATHPLANNING\n"
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=ExplicitDefaultsHelpFormatter)

    parser.add_argument("-s", "--source", help="heightmaps source to use", choices=list(Source), type=Source, default=Source.RTDB)
    parser.add_argument("-m", "--heightmap", help="which heightmap to edit")

    group = parser.add_argument_group("rtdb options")
    group.add_argument("-p", "--path", help="database path to use", type=Path, default=Path(RTDB2_DEFAULT_PATH))
    group.add_argument("-a", "--agent", help="agent ID to use", type=int, default=rtdb2tools.guessAgentId())
    group.add_argument("-k", "--key", help="RtDB key to use", type=str, default="CONFIG_TEAMPLAY")

    group = parser.add_argument_group("config file options")
    group.add_argument("-f", "--file", help="config file to use", type=Path, default=Path(falconspy.FALCONS_CONFIG_PATH, "teamplay.yaml"))

    group = parser.add_argument_group("other operations")
    group.add_argument("-e", "--export", help="save heightmap to this source and exit", choices=list(Source), type=Source)

    args = parser.parse_args()

    def instantiate_handler(source):
        if source == Source.RTDB:
            p = args.path / str(args.agent) / "default"
            return RtDBDataHandler(p, args.agent, args.key)

        if source == Source.CONFIG:
            return TeamplayConfigHandler(args.file)

        if source == Source.DUMMY:
            return DummyHandler()

    def create_handler(source):
        handler = instantiate_handler(source)
        handler.load()

        for key in ["heightmaps", "factors"]:
            handler.select(key)

        return handler

    handler = create_handler(args.source)

    # Export heightmap data from one source to another
    if args.export:
        export_handler = create_handler(args.export)

        if args.heightmap:
            handler.select(args.heightmap)
            export_handler.select(args.heightmap)

        export_handler.data = handler.data
        export_handler.save()
        sys.exit(0)

    # Run editor GUI
    App = HeightmapEditorApp()
    App.handler = handler
    App.heightmap = args.heightmap
    App.run()
