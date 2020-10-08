 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "ext/falconsCommonEnv.hpp"

#include <string>
#include <stdlib.h>

bool isSimulatedEnvironment()
{
    bool isSimulated = false;
    std::string env_simulated;
    try
    {
        char* env_sim = getenv("SIMULATED");
        env_simulated = std::string(env_sim);
    }
    catch (...)
    {
        //std::cout << "Environment variable SIMULATED not found." << std::endl; // suppress spam
        // this is an error state, suggesting something weird in scripting or stand-alone deployment
        // hence we SET the simulated flag, so at least traffic will remain local
        isSimulated = true;
    }

    if (env_simulated == "1")
    {
        isSimulated = true;
    }

    return isSimulated;
}

int getRobotNumber()
{
    int robotNumber = 0;

    if (getenv("TURTLE5K_ROBOTNUMBER") != NULL)
    {
        robotNumber = atoi(getenv("TURTLE5K_ROBOTNUMBER"));
    }

    return robotNumber;
}

bool isGoalKeeper()
{
    // TODO this should not depend on robotNumber; what if r2 decides to behave as goalKeeper?
    // definition is vague anyway --
    // are we interested in hardware aspects (do we have a frame, then let's ignore some fake obstacles?)
    // or in software / behavior (behave as goalKeeper?)
    return (getRobotNumber() == 1);
}

char getTeamChar() // A or B
{
    char teamChar = 'A';

    if (getenv("TURTLE5K_TEAMNAME") != NULL)
    {
        if (getenv("TURTLE5K_TEAMNAME") == std::string("teamB"))
        {
            teamChar = 'B';
        }
    }

    return teamChar;
}

