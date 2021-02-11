// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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


bool process_command_line(int argc, char** argv, int& agentId, int& verbosity, std::string& inputFile, std::string& outputFile, 
                          int& overruleRobotPos, std::vector<std::string>& overruledKeys, std::vector<std::string>& deletedKeys)
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
            ("overrule_robot_pos,p", po::value<int>(&overruleRobotPos)->default_value(0), "overrule robot position with position already on rdl")
            ("overrule_keys,k", po::value<std::vector<std::string> >()->multitoken()->zero_tokens()->composing(), "overrule keys on the new frame with values from existing frame")
            ("delete_keys,d", po::value<std::vector<std::string> >()->multitoken()->zero_tokens()->composing(), "keys to be deleted from the existing frame")
            
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

        if(vm.count("overrule_keys"))
        {
            overruledKeys = vm["overrule_keys"].as<std::vector<std::string> >();
        }

        if(vm.count("delete_keys"))
        {
            deletedKeys = vm["delete_keys"].as<std::vector<std::string> >();
        }        
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
    int overruleRobotPos = 0;
    std::vector<std::string> overruledKeys;
    std::vector<std::string> deletedKeys;
    bool result = process_command_line(argc, argv, agentId, verbosity, inputFile, outputFile, overruleRobotPos, overruledKeys, deletedKeys);
    if (!result)
    {
        return 1;
    }

    // initialize and run
    // TODO: override configuration yaml file?
    cWorldModelStimulator stim(agentId, inputFile, outputFile);
    stim.setVerbosity(verbosity);
    stim.setRobotPosOverrule(overruleRobotPos == 1);
    for(auto it = overruledKeys.begin(); it != overruledKeys.end(); it++)
    {
        stim.add_overruled_key(*it);
    }
    for(auto it = deletedKeys.begin(); it != deletedKeys.end(); it++)
    {
        stim.add_deleted_key(*it);
    }
    

    stim.run();

    return 0;
}

