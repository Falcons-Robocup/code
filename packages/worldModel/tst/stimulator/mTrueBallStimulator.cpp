// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mTrueBallStimulator.cpp
 *
 *  Created on: Aug 2020
 *      Author: Lucas Catabriga
 */


// system includes
#include <iostream>
#include <boost/program_options.hpp>

// internal includes
#include "cTrueBallStimulator.hpp"

// other packages
#include "falconsCommon.hpp"

// local defines and namespaces
namespace po = boost::program_options;


bool process_command_line(int argc, char** argv, int& agentId, int &verbosity, std::string& inputFile, std::string& outputFile, int& flipped)
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
            ("flip,f", po::value<int>(&flipped)->default_value(0), "set verbosity level")
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
    int flip;
    bool result = process_command_line(argc, argv, agentId, verbosity, inputFile, outputFile, flip);
    if (!result)
    {
        return 1;
    }

    // initialize and run
    // TODO: override configuration yaml file?
    cTrueBallStimulator stim(agentId, inputFile, outputFile, flip);
    stim.setVerbosity(verbosity);
    stim.run();

    return 0;
}

