 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "ext/falconsCommonDirs.hpp"

#include <string>
#include <stdlib.h>
#include <iostream>

// internal helper function: get environment variable or error
std::string getEnvWrapper(std::string key);

std::string pathToFalconsRoot()
{
    // typically /home/robocup/falcons
    return getEnvWrapper("FALCONS_PATH");
}

std::string pathToCodeRepo()
{
    // typically /home/robocup/falcons/code
    return getEnvWrapper("FALCONS_CODE_PATH");
}

std::string pathToConfig()
{
    // typically /home/robocup/falcons/code/config
    return getEnvWrapper("FALCONS_CODE_PATH") + "/config";
}

std::string pathToScripts()
{
    // typically /home/robocup/falcons/code/scripts
    return getEnvWrapper("FALCONS_SCRIPTS_PATH");
}

std::string pathToTeamplayDataRepo()
{
    // typically /home/robocup/falcons/teamplayData
    return getEnvWrapper("FALCONS_TPDATA_PATH");
}

std::string pathToDataRepo()
{
    // typically /home/robocup/falcons/data
    return getEnvWrapper("FALCONS_DATA_PATH");
}


// internal helper function: get environment variable or error
std::string getEnvWrapper(std::string key)
{
    char *cp = NULL;
    if ((cp = getenv(key.c_str())) == NULL)
    {
        std::cerr << "ERROR: missing environment variable: " + key << std::endl;
        return "ERROR";
    }

    return cp;
}

