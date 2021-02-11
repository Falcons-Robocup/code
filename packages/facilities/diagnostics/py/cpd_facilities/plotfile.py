# Copyright 2020 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import os
import argparse

import falconspy
import rdlLib
import plotdata

class PlottableTextFile:
    def __init__(self, plot_data_enum, robot_id, output_filename):
        """
        """

        # Ensure the directory exists where to write the output_filename
        os.makedirs(os.path.dirname(output_filename), exist_ok=True)

        # data_fields = {key1: [field1, field2], key2: [field1, field2]}
        self.data_fields = plotdata.plotDataMapping[ plot_data_enum ]
        self.robot_id = robot_id
        self.output_filename = output_filename

    ####################
    # PUBLIC FUNCTIONS #
    ####################

    def writeFileFromRDL(self, rdl_filename, ageMin, ageMax):
        rdl_file = rdlLib.RDLFile(rdl_filename)
        rdl_file.parseRDL(ageMin, ageMax)

        with open(self.output_filename, "w") as f:
            f.write( self._makePlottableTextFileHeader() )

            for rdl_frame in rdl_file.frames:
                if self.robot_id in rdl_frame.data:

                   text_file_line = self._makePlottableTextFileLineFromRDLFrame(rdl_frame)

                   if text_file_line is not None:
                       f.write( text_file_line )


    #####################
    # PRIVATE FUNCTIONS #
    #####################

    def _makePlottableTextFileHeader(self):
        """
        Create header for plottable textfile
        e.g.,
        self.data_fields = {key1: [field1, field2], key2: [field1, field2]}
        =>
        "age key1.field1 key1.field2 key2.field1 key2.field2"
        """
        
        data_headers = []
        for key in self.data_fields:
            for field in self.data_fields[key]:
                data_headers.append( key + "." + field )

        return "age;" + ";".join(data_headers) + "\n"

    def _makePlottableTextFileLineFromRDLFrame(self, rdl_frame):
        """
        Translates a single RDLFrame into a single line for the plottable textfile
        e.g.,
        "23.3 0.0 0.0 0.0"
        """
        
        data_fields = []
        for key in self.data_fields:

            # if "DIAG_PERIPHERALSINTERFACE" in frame.data[2]:
            if key in rdl_frame.data[self.robot_id]:

                # key_data = frame.data[3]["DIAG_PERIPHERALSINTERFACE"]
                key_data = rdl_frame.data[self.robot_id][key]

                for value in self.data_fields[key]:

                    try:
                        # key_data.value['speed_vel'][0]
                        value_data = eval( "key_data.value" + value )
                    except:
                        print("Error: Unable to resolve the value '%s' in key '%s' for robot_id '%d'" % (value, key, self.robot_id))
                        exit()

                    data_fields.append( str(value_data) )

            else:
                for value in self.data_fields[key]:
                    # append 'empty' values
                    data_fields.append( " " )

        return "%s;%s\n" % (rdl_frame.age, ";".join(data_fields))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script generates a plottable textfile")
    parser.add_argument("--robotid", type=int, required=True, help="robotid. Example: --robotid 3")
    parser.add_argument("--data", type=str, required=True, help="data enum. Example: --data PlotData.MOTION")
    parser.add_argument("--rdl_filename", type=str, required=True, help="rdl filename. Example: --rdl_filename /path/to/rdl")
    parser.add_argument("--ageMin", type=float, required=True, help="ageMin. Example: --ageMin ageMin")
    parser.add_argument("--ageMax", type=float, required=True, help="ageMax. Example: --ageMax ageMax")
    parser.add_argument("--output_filename", type=str, required=True, help="output filename. Example: --filename /path/to/textfile")
    args = parser.parse_args()

    plot_data_enum = eval( "plotdata." + args.data )

    textfile = PlottableTextFile(plot_data_enum, args.robotid, args.output_filename)
    textfile.writeFileFromRDL(args.rdl_filename, args.ageMin, args.ageMax)
