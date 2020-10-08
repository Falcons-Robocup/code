 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "ext/falconsCommonDirs.hpp"
#include "ext/falconsCommonEnv.hpp"
#include "ext/falconsCommonConfig.hpp"

#include <string>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>


std::string determineConfig(std::string key, std::string fileExtension)
{
    // determine config folder
    std::string cfgFolder = pathToConfig();
    // determine yaml file to load
    std::string target = key;
    if (isSimulatedEnvironment()) // simulator override?
    {
        std::string candidate = key + "Sim";
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
        // test robot-specific
        candidate = key + "SimR" + boost::lexical_cast<std::string>(getRobotNumber());
        candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
    }
    else // not simulator
    {
        // test robot-specific
        std::string candidate = key + "R" + boost::lexical_cast<std::string>(getRobotNumber());
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
    }
    // final check and return
    std::string result = cfgFolder + "/" + target + fileExtension;
    if (!boost::filesystem::exists(result))
    {
        printf("ERROR: could not resolve yaml file for %s\n", key.c_str());
    }
    else
    {
        printf("resolved yaml file %s -> %s\n", key.c_str(), result.c_str());
    }
    return result;
}

