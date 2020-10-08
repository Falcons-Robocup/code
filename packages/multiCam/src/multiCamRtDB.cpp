 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 
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
