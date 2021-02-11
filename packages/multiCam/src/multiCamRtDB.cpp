// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#include "multiCam.hpp"

#include <boost/lexical_cast.hpp>

#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"
#include "observerRtDB.hpp"
#include "tracing.hpp"

using namespace cv;
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
    INIT_TRACE;

    int opt = 0;
    bool guiEnabled = true;

    int robotIdArg = 0;
    float frequency = 40.0;
    while( (opt = getopt(argc, argv, "ci:f:") ) != -1 )
    {
        switch(opt)
        {
        	case 'c':
        		guiEnabled = false;
        		printf("INFO      : disable the GUI\n");
        		break;
        	case 'i':
        		robotIdArg = atoi(optarg);
        		printf("INFO      : using command argument to set robot id to %d\n", robotIdArg);
        		break;
            case 'f':
                frequency = atof(optarg);
                printf("INFO      : using command argument to set frequency to %.1f\n", frequency);
                break;
        }
    }

    multiCamLibrary *multCam = new multiCamLibrary( robotIdArg, guiEnabled );

    // attach the observer
    bool cameraCorrectlyMounted = false; // TODO remove, obsolete
    observerRtDB *observerObj = new observerRtDB(robotIdArg, cameraCorrectlyMounted, 2.5, frequency);
    multCam->attach(observerObj);

    // run forever and without any sleep / timed loop, because library
    // will wait for and directly operate on new cam data
    while (multCam->update()) {}

    return 0;
}
