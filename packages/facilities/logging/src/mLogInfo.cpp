// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mLogInfo.cpp
 *
 * Write log info to stdout.
 *
 *  Created on: January 2019
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileReader.hpp"
#include <iostream>
#include <fstream>
#include <set>


std::ifstream::pos_type filesize(const char* filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
}


int main(int argc, char **argv)
{
    // need argument
    if (argc == 1)
    {
        std::cerr << "ERROR: no logfile given" << std::endl;
        return 1;
    }

    // help request?
    if (std::string(argv[1]) == std::string("-h"))
    {
        std::cout << "usage: rdlinfo [-h] rdlfile" << std::endl;
        std::cout << std::endl;
        std::cout << "Show info and statistics on given RDL logging file." << std::endl;
        return 0;
    }

    // prepare
    cLogFileReader logFile(argv[1]);
    tLogHeader header = logFile.getHeader();
    int numFrames = 0;
    tLogFrame frame;

    // run
    int totalDataSize = 0;
    int minDataSize = 999999;
    int maxDataSize = 0;
    float maxAge = 0;
    bool ok = true;
    while (ok)
    {
        try
        {
            ok = logFile.getFrame(frame);
        }
        catch (...)
        {
            std::cerr << "ERROR at frame " << numFrames << std::endl;
            ok = false;
        }
        if (!ok)
        {
            break;
        }
        numFrames++;
        // inspect frame size
        int b = frame.data.size();// + 4; // 4 = string size descriptor
        if (b < minDataSize)
        {
            minDataSize = b;
        }
        if (b > maxDataSize)
        {
            maxDataSize = b;
        }
        totalDataSize += b;

        // store max age (age since first frame)
        if (frame.age > maxAge)
        {
            maxAge = frame.age;
        }
    }

    // display basic statistics
    std::cout << "     filename: " << header.filename << std::endl; // does not have to be consistent with actual filename
    int sz = filesize(argv[1]);
    float mb = sz / 1024.0 / 1024.0;
    std::cout << "     filesize: " << sz << " [B] (" << std::fixed << std::setprecision(2) << mb << "MB)"<< std::endl;
    std::cout << "     hostname: " << header.hostname << std::endl;
    std::cout << "     creation: " << header.creation.toStr() << std::endl;
    std::cout << "     duration: " << std::fixed << std::setprecision(2) << maxAge << " [s]" << std::endl;
    std::cout << "    frequency: " << header.frequency << " [Hz]" << std::endl;
    std::cout << "   compressed: " << (header.compression ? "yes" : "no") << std::endl;
    std::cout << "    numFrames: " << numFrames << std::endl;

    // display advanced statistics
    std::cout << "  minDataSize: " << minDataSize << " [B]" << std::endl;
    std::cout << "  maxDataSize: " << maxDataSize << " [B]" << std::endl;
    std::cout << "  avgDataSize: " << std::fixed << std::setprecision(2) << (totalDataSize / 1.0 / numFrames) << " [B/frame]" << std::endl;
    std::cout << " avgFrameSize: " << std::fixed << std::setprecision(2) << (sz / 1.0 / numFrames) << " [B/frame]" << std::endl;
    std::cout << "  avgOverhead: " << std::fixed << std::setprecision(2) << ((sz - totalDataSize) / 1.0 / numFrames) << " [B/frame]" << std::endl;
    std::cout << "  avgDataRate: " << std::fixed << std::setprecision(2) << (totalDataSize / 1024.0 / maxAge) << " [KB/s]" << std::endl;
    return 0;
}

