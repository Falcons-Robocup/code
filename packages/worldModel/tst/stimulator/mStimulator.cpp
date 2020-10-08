 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mStimulator.cpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 */


// system includes
#include <iostream>
#include <boost/program_options.hpp>

// internal includes
#include "cWorldModelStimulator.hpp"

// other packages
#include "falconsCommon.hpp"

// local defines and namespaces
namespace po = boost::program_options;


bool process_command_line(int argc, char** argv, int& agentId, int &verbosity, std::string& inputFile, std::string& outputFile)
{
    try
    {
        po::options_description desc("worldModelStimulator options");
        desc.add_options()
            ("help,h", "produce help message")
            ("agent,a", po::value<int>(&agentId)->default_value(0), "set agent id")
            ("input,i", po::value<std::string>(&inputFile)->required(), "set the input file")
            ("output,o", po::value<std::string>(&outputFile)->default_value("auto"), "set the output file")
            ("verbosity,v", po::value<int>(&verbosity)->default_value(1), "set verbosity level")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return false;
        }

        // https://stackoverflow.com/questions/5395503/required-and-optional-arguments-using-boost-library-program-options
        // There must be an easy way to handle the relationship between options "help" and others
        // Yes, the magic is putting the po::notify after "help" option check
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    // option parsing
    int agentId = 0;
    int verbosity = 1;
    std::string inputFile;
    std::string outputFile;
    bool result = process_command_line(argc, argv, agentId, verbosity, inputFile, outputFile);
    if (!result)
    {
        return 1;
    }

    // initialize and run
    // TODO: override configuration yaml file?
    cWorldModelStimulator stim(agentId, inputFile, outputFile);
    stim.setVerbosity(verbosity);
    stim.run();

    return 0;
}

