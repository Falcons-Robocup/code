# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3
import os, sys
import shutil
import threading
import math
import argparse
import pykst as kst
import time

import falconspy
import kstplot_lib
import rtdb2tools


def PathPlanningDataAdapter(writer):
    # ROBOT_VELOCITY_SETPOINT is a list of three numbers: (vx, vy, vRz)
    writer.subscribe("ROBOT_VELOCITY_SETPOINT", lambda v: "%10.4f %10.4f %10.4f" % (v[0], v[1], v[2]))
    writer.output("timestamp.setpoint velocity.setpoint.x velocity.setpoint.y velocity.setpoint.Rz # header line", "ROBOT_VELOCITY_SETPOINT")
    # ROBOT_STATE (velocity FCS, so (vx,vy) need to be transformed to RCS)
    def transform(velocity, robotRz):
        # copied from FalconsCoordinates.py in packages/geometry
        angle = (math.pi*0.5 - robotRz)
        s = math.sin(angle)
        c = math.cos(angle)
        return (c * velocity[0] - s * velocity[1], s * velocity[0] + c * velocity[1], velocity[2])
    def formatterFloat(values):
        if not isinstance(values, list) and not isinstance(values, tuple):
            values = [values]
        return " ".join(("%10.4f" % v) for v in values)
    def formatterInt(values):
        if not isinstance(values, list):
            values = [values]
        return " ".join(("%d" % v) for v in values)
    writer.subscribe("ROBOT_STATE", lambda v: formatterFloat(transform(v[3], v[2][2])) + " " + str(int(v[4])))
    writer.output("timestamp.robotstate velocity.robotstate.x velocity.robotstate.y velocity.robotstate.Rz hasBall # header line", "ROBOT_STATE")
    # DIAG_PATHPLANNING distance to sub-target
    writer.subscribe("DIAG_PATHPLANNING", lambda v:
        formatterFloat(v["distanceToSubTargetRCS"]) + " " +
        formatterFloat(v["accelerationRCS"]) + " " +
        formatterInt(v["isAccelerating"]) + " " +
        formatterInt(v["accelerationClipping"]) + " " +
        "-" + str(int(v["shortStroke"])) + " " + # minus sign for visualization ...
        formatterInt(v["deadzone"]) + " " +
        formatterFloat(v["pid"]["x"]["proportional"]) + " " +
        formatterFloat(v["pid"]["x"]["integral"]) + " " +
        formatterFloat(v["pid"]["x"]["derivative"]) + " " +
        formatterFloat(v["pid"]["y"]["proportional"]) + " " +
        formatterFloat(v["pid"]["y"]["integral"]) + " " +
        formatterFloat(v["pid"]["y"]["derivative"]) + " " +
        formatterFloat(v["pid"]["Rz"]["proportional"]) + " " +
        formatterFloat(v["pid"]["Rz"]["integral"]) + " " +
        formatterFloat(v["pid"]["Rz"]["derivative"]))
    writer.output("timestamp.diagnostics distance.subtarget.x distance.subtarget.y distance.subtarget.Rz acceleration.x acceleration.y acceleration.Rz accelerating.x accelerating.y accelerating.Rz acceleration.clipping.x acceleration.clipping.y acceleration.clipping.Rz shortStroke deadzone.x deadzone.y deadzone.Rz pid.x.proportional pid.x.integral pid.x.derivative pid.y.proportional pid.y.integral pid.y.derivative pid.Rz.proportional pid.Rz.integral pid.Rz.derivative # header line", "DIAG_PATHPLANNING")
    # TODO: CONFIG_PATHPLANNING tolerances?
    return writer


class KSTSession():
    def __init__(self, showAge=10.0):
        self.showAge = showAge
        # window and application will start immediately and run in background
        self.client = kst.Client("kst_pathPlanning")
        kstfile = falconspy.FALCONS_CODE_PATH + "/packages/facilities/diagnostics/kst/pathPlanning.kst" # TODO: make relative?
        # use a tmp file to prevent accidental overwriting if user presses SAVE
        tmpKstFile = "/tmp/pathPlanning.kst"
        shutil.copyfile(kstfile, tmpKstFile)
        self.client.open_kst_file(tmpKstFile)
        # extract some common-used things (rather than doing it every iteration)
        self.plotX = self.client.plot('P1') # X plot

    def run(self):
        """
        Low frequent GUI refresh upon data change.
        """
        # we monitor timestamp vector to detect data change
        # in playback pause mode this is handy, because it allows the user to zoom / do whatever in kst GUI
        # but once playback resumes, fixed scaling is applied again
        frequency = 1.0 # don't set too high, client calls are slow ...
        dt = 1.0 / frequency
        tEndPrev = 0
        while True:
            # get timestamp
            try:
                v = self.client.data_vector('timestamp.worldstate')
                tEnd = v.get_numpy_array()[-1]
            except:
                break # probably GUI window was closed
            if tEnd > tEndPrev:
                self.set_fixed_x_range_from_end(tEnd)
                # dynamic boxes do not nicely work, the interface to KST client is way too slow
                #self.clear_all_boxes()
                #self.highlight_tokyo_drift()
            # sleep
            time.sleep(dt)
            tEndPrev = tEnd

    def set_fixed_x_range_from_end(self, tEnd):
        """
        Default overrule: set X axis range fixed to last few seconds.

        This is not possible with manual settings - there is an option to 'read n frames from end'
        but that messes up the data vector co-plotting ... too bad it is not a visualization option ...
        (if user wants to get the big picture, just press M in the GUI)
        """
        if self.showAge != None:
            tStart = tEnd - self.showAge
            self.plotX.set_x_range(tStart, tEnd)
            # this will automatically rescale all windows, since we force-tied them in .kst file configuration

    def clear_all_boxes(self):
        boxes = self.client.get_box_list()
        for b in boxes:
            b.remove()

    def highlight_tokyo_drift(self):
        # calculate periods
        periods = self.calculate_periods('timestamp.diagnostics', 'tokyodrift', lambda v: v == 1)
        # draw in each of 3 plots
        for p in periods:
            t = p[0]
            duration = p[1]
            for ifig in range(3):
                b = self.client.new_box(pos=(0.1,0.2+0.3*ifig), fill_style=0, stroke_brush_color='#ff55ff', stroke_width=1, stroke_style=3)
                b.set_parent_auto()
                b.set_lock_pos_to_data(True)
                b.set_pos((t + 0.5 * duration, 0.0))
                b.set_size((duration, 2.5))

    def calculate_periods(self, tname, vname, predicate):
        # get vectors
        t = self.client.data_vector(tname).get_numpy_array()
        p = [predicate(v) for v in self.client.data_vector(vname).get_numpy_array()]
        # iterate
        return getPhases(t, p)


# Main structure of the program
if __name__ == "__main__":

    # Argument parsing.
    descriptionTxt = """Plots pathPlanning-related parameters of one robot, specifically:
    * distance to current sub-target
    * velocity setpoint (RCS)
    * velocity feedback (RCS) [fallback?]

Note: this tool can only run for one robot at a time.
"""
    exampleTxt = """Examples:
    kstplot_pp -a 4 `newestRDL.py`
    kstplot_pp -a 5
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-a', '--agent', help='agent ID to use', type=int, default=kstplot_lib.rtdb2tools.guessAgentId())
    parser.add_argument('-k', '--kstonly', help='do not reset and monitor diagnostics files, instead use existing', action='store_true') # useful for KST visualization development
    parser.add_argument('-f', '--tmpfolder', help='temporary working folder to use', default='/var/tmp/kst_pathPlanning')
    parser.add_argument('-r', '--remote', help='monitor remote robot', action='store_true')
    parser.add_argument('rdlfile', help='extract data from RDL file instead of live RTDB', default=None, nargs='?')
    args = parser.parse_args()

    # Check if robot id was provided or successfully guessed
    if args.agent == -1 and not args.kstonly:
        raise Exception("invalid agent id, expected an int >= 0, got " + str(args.agent))

    # TODO: to allow users to instantiate multiple robot plotters simultaneously, we would have to
    # * specialize the work folder (append '_r6' for instance)
    # * fix reference to it in the .kst file, preferably dynamically

    # Setup and run data adapter
    if not args.kstonly:
        if args.rdlfile == None:
            if args.remote:
                # Start the monitor on remote robot
                raise Exception("remote mode not yet implemented")
                # TODO command remote
                # TODO keep file in sync (tail -f ?) only a few Kb/s
            else:
                # Live mode
                writer = kstplot_lib.LiveDiagnosticsMonitor(agent=args.agent, folder=args.tmpfolder)
                writer = PathPlanningDataAdapter(writer)
                # temporary HACK ... for older .rdls without this data, the file will only contain header line
                # and it will cause the x-axis in KSTplot to be totally screwed (hard linked to t=0!)
                # so as a hack, we provide a dummy line with normal timestamp and zero data ....
                t = kstplot_lib.rtdb2tools.RtDBTime()
                t.now()
                ts = str(t)
                writer.output(ts + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", "DIAG_PATHPLANNING")
                # Run
                t = threading.Thread(target=writer.run)
                t.start()
        else:
            # RDL mode
            print("loading " + args.rdlfile, newline=False)
            writer = kstplot_lib.RDLDiagnostics(args.rdlfile, args.agent, args.tmpfolder)
            writer = PathPlanningDataAdapter(writer)
            writer.run()
            print(" done")
    else:
        if args.rdlfile != None:
            raise Exception("option 'kstonly' does not work in combination with RDL file")

    # Setup and run KST session
    # (kind of equivalent to command-line 'kst2 pathPlanning.kst', but with more features)
    showAge = None
    if not args.kstonly and args.rdlfile == None:
        showAge = 20.0
        k = KSTSession(showAge)
        try:
            k.run()
        except KeyboardInterrupt:
            pass
    else:
        k = KSTSession(showAge)

    # Cleanup
    if not args.kstonly and args.rdlfile == None:
        writer.done = True

