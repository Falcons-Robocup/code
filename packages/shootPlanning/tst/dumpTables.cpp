 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

