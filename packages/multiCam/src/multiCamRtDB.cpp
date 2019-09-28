 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TKOV: i added the multiCam.hpp over here, but i am not sure if an additinal path is required
// include for the multiCam class
#include "multiCam.hpp"

/* ROS global includes */
#include <boost/lexical_cast.hpp>

/* ROS node-specific includes */

#include "FalconsCommon.h"
#include "cDiagnostics.hpp"
#include "observerRtDB.hpp"

using namespace cv;
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
    int opt = 0;
    bool guiEnabled = true;

    int robotIdArg = 0;
    while( (opt = getopt(argc, argv, "ci:") ) != -1 )
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
        }
    }

    multiCamLibrary *multCam = new multiCamLibrary( robotIdArg, guiEnabled );

    /* Take care of ROS initialization */
    ros::init(argc, argv, "multiCam");

    // attach the observer
    bool cameraCorrectlyMounted = false; // but for balls and obstacles it needs to be true?!?! TODO Andre please help, workaround for now in observerRos.cpp
    observerRtDB *observerObj = new observerRtDB(robotIdArg, cameraCorrectlyMounted, 5.0);
    multCam->attach(observerObj);

    // TKOV: can you please check the section below?
    // i am not sure if these things are still required, or
    // otherwise maybe i deleted to much.
    // From multiCam point of view only multCam->update() is required.
    // If needed check multiCamAlone.cpp as reference

    float frequency = 30.0;
    rtime::loop(frequency, boost::bind(&multiCamLibrary::update, multCam));
    
    return 0;
}
