// Copyright 2017-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * dumpTables.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: Jan Feitsma
 *
 * Dump the shoot tables.
 */


#include "int/cShotSolver.hpp"
#include <stdio.h>


int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        fprintf(stderr, "ERROR: require calibration file as argument\n");
        return 1;
    }
    printf("constructing lookup table\n");
    fflush(stdout);
    cShotSolver solver;
    solver.load(argv[1]);

    float strength, angle;
    
    for (int doLob = 0; doLob <= 1; ++doLob)
    {
        FILE *fp1 = NULL, *fp2 = NULL;
        std::string filename1, filename2; 
        if (doLob)
        {
            filename1 = "/var/tmp/lobShotsPower.txt";
            filename2 = "/var/tmp/lobShotsAngle.txt";
            printf("dumping lob shot tables\n");
        }
        else
        {
            filename1 = "/var/tmp/straightShotsPower.txt";
            filename2 = "/var/tmp/straightShotsAngle.txt";
            printf("dumping straight shot tables\n");
        }
        fp1 = fopen(filename1.c_str(), "w");
        fp2 = fopen(filename2.c_str(), "w");
        fflush(stdout);
        bool first = true;
        for (float distance = 1.0; distance <= 10.0; distance += 1e-2)
        {
            if (first)
            {
                fprintf(fp1, "     0");
                fprintf(fp2, "     0");
                for (float z = 0.0; z <= 2.0; z += 1e-2)
                {
                    fprintf(fp1, " %6.2f", z);
                    fprintf(fp2, " %6.2f", z);
                }
                fprintf(fp1, "\n");
                fprintf(fp2, "\n");
                first = false;
            }
            fprintf(fp1, "%6.2f", distance);
            fprintf(fp2, "%6.2f", distance);
            for (float z = 0.0; z <= 2.0; z += 1e-2)
            {
                bool r = solver.query(distance, z, (bool)doLob, strength, angle);
                if (r)
                {
                    fprintf(fp1, " %6.2f", strength);
                    fprintf(fp2, " %6.2f", angle);
                }
                else
                {
                    fprintf(fp1, "      0");
                    fprintf(fp2, "      0");
                }
            }
            fprintf(fp1, "\n");
            fprintf(fp2, "\n");
        }
        printf("file written: %s\n", filename1.c_str());
        printf("file written: %s\n", filename2.c_str());
        fclose(fp1);
        fclose(fp2);
    }
    return 0;
}

