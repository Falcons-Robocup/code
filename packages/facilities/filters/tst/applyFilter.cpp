// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * applyFilter.cpp
 *
 *  Created on: Mar 2020
 *      Author: Jan Feitsma
 */


// system includes
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

// internal includes
#include "ext/LinearInterpolator.hpp"
#include "ext/LeastSquaresInterpolator.hpp"

// other packages
//#include "falconsCommon.hpp"

// local defines and namespaces
namespace po = boost::program_options;


bool process_command_line(int argc, char** argv, std::string& filterType, std::string& inputFile, std::string& outputFile)
{
    try
    {
        po::options_description desc("applyFilter options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filter,f", po::value<std::string>(&filterType)->required(), "set the filter type")
            ("input,i", po::value<std::string>(&inputFile)->required(), "set the input file")
            ("output,o", po::value<std::string>(&outputFile)->default_value("/tmp/applyFilterOutput.txt"), "set the output file")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return false;
        }
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
    std::string filterType;
    std::string inputFile;
    std::string outputFile;
    bool result = process_command_line(argc, argv, filterType, inputFile, outputFile);
    if (!result)
    {
        return 1;
    }

    // handle remaining options (after '--')
    std::vector<std::string> extra_args;
    bool ignore = true;
    for (int it = 0; it < argc; ++it)
    {
        if (!ignore)
        {
            extra_args.push_back(argv[it]);
        }
        if (std::string(argv[it]) == std::string("--"))
        {
            ignore = false;
        }
    }

    // construct applicable filter using optional arguments
    AbstractInterpolator *filter = NULL;
    if (filterType == "lsq")
    {
        if (extra_args.size() < 2)
        {
            std::cerr << "Too few arguments for filter 'lsq'" << std::endl;
            return 1;
        }
        int order = 0;
        float range = 0.0;
        try
        {
            order = boost::lexical_cast<int>(extra_args[0]);
            range = boost::lexical_cast<float>(extra_args[1]);
        }
        catch(...)
        {
            std::cerr << "Failed to convert argument(s) for filter 'lsq'" << std::endl;
            return 1;
        }
        filter = new LeastSquaresInterpolator(order, range);
    }
    else if (filterType == "linear")
    {
        // no arguments
        filter = new LinearInterpolator();
    }
    else
    {
        std::cerr << "Invalid filter type '" << filterType << "', expect one of ('linear', 'lsq')" << std::endl;
        return 1;
    }

    // feed data from input file to the filter
    std::ifstream infile(inputFile);
    std::string line;
    float xmin = 1e9, xmax = 0.0;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        float x, y;
        iss >> x >> y;
        filter->feed(x, y);
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
    }
    infile.close();

    // write samples to output file
    float xstep = (xmax - xmin) / 1000.0;
    filter->writeInterpolatedCurve(xmin, xmax, xstep, outputFile);
    std::cout << "File written: " << outputFile << std::endl;

    // cleanup
    delete(filter);

    return 0;
}

