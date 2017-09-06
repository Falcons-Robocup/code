 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testLocalizationIO.cpp
 *
 *  Created on: Jul 08, 2017
 *      Author: Jan Feitsma
 */

#include <gtest/gtest.h>
#include <glob.h>

#include "int/administrators/localizationStimulator.hpp"

#include "FalconsCommon.h"


std::string tstdir = "/home/robocup/falcons/code/packages/worldModel/tst/localization";




// Parametrization
typedef std::string TestParam;
class LocalizationFileIOTest : public ::testing::TestWithParam<TestParam>
{
};

// Test cases - parameterized
TEST_P(LocalizationFileIOTest, testFileIO)
{
    std::string filename = GetParam();
    TRACE("LocalizationFileIOTest(%s) start", filename.c_str());

    // setup
    localizationStimulator stim;
    stim.load(filename);
    
    // execute
    stim.run();
    
    // verify
    EXPECT_TRUE(stim.verify());
    
    TRACE("LocalizationFileIOTest(%s) end", filename.c_str());
}

// Discover all files
std::vector<TestParam> listFiles(std::string tstdir)
{
    std::vector<std::string> result;
    glob_t globbuf;
    int err = glob((tstdir + "/*.txt").c_str(), 0, NULL, &globbuf);
    TRACE("err=%d numfound=%d", err, globbuf.gl_pathc);
    if (err == 0)
    {
        for (size_t i = 0; i < globbuf.gl_pathc; i++)
        {
            result.push_back(globbuf.gl_pathv[i]);
        }
        globfree(&globbuf);
    }
    // sort
    std::sort(result.begin(), result.end());
    return result;
}

// Instantiate for each file
INSTANTIATE_TEST_CASE_P(LocalizationFileIOTest, LocalizationFileIOTest, testing::ValuesIn(listFiles(tstdir)));

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

