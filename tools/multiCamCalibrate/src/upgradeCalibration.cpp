/*
 * upgradeCalibration.cpp
 *
 * Conversion command-line utility for Optical Calibration.
 * Given 'old' calibration files, which do not yet have front view lookup tables for the angles
 * (used in ball detection), calculate and add those tables.
 *
 *  Created on: May 2018
 *      Author: Jan Feitsma
 */


#include "optiCal.hpp"
#include <string>


void doit(std::string const &filename1, std::string const &filename2)
{
    opticalCalibrator optical;
    optical.load(filename1);
    optical.upgrade();
    optical.save(filename2);
}


int main(int argc, char **argv)
{

    // check number of given arguments
    if (argc == 2)
    {
        // in place
        doit(argv[1], argv[1]);
    }
    else if (argc == 3)
    {
        // copy and extend
        doit(argv[1], argv[2]);
    }
    else
    {
        fprintf(stderr, "ERROR: require at least one filename as argument");
        return 1;
    }

    return 0;
}

