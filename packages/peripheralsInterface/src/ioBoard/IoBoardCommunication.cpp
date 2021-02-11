// Copyright 2017-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IoBoard.cpp
 *
 *    Created on: Apr 25, 2017
 *            Author: Edwin Schreuder
 */

#include <algorithm>
#include <iostream>
#include <exception>

#include <cDiagnostics.hpp>
#include <falconsCommon.hpp>
#include "tracing.hpp"

#include "int/ioBoard/IoBoardCommunication.hpp"

using std::array;
using std::copy;
using std::endl;
using std::exception;
using std::find;
using std::string;

const unsigned char sof = 0x5a;
const size_t maxRetries = 2;

IoBoardCommunication::IoBoardCommunication(string portName) :
                serial(portName, B115200, 0.1, 0)
{
    TRACE(">");
    TRACE("<");
}

IoBoardCommunication::~IoBoardCommunication()
{
    TRACE(">");
    TRACE("<");
}

void IoBoardCommunication::set(unsigned char command, Data data)
{
    TRACE(">");

    do_transaction(command, data);

    TRACE("<");
}

IoBoardCommunication::Data IoBoardCommunication::get(unsigned char command)
{
    TRACE(">");

    Data result;
    result = do_transaction(command, IoBoardCommunication::Data());

    TRACE("<");
    return result;
}

#include <iostream>

IoBoardCommunication::Data IoBoardCommunication::do_transaction(unsigned char command, Data data)
{
    TRACE(">");

    bool transaction_complete = false;
    Package package = {command, data};

    for (size_t retries = 0; (retries < maxRetries) && !transaction_complete; retries++)
    {
        try
        {
            package = transceive_package(package);
            transaction_complete = true;
        }
        catch (exception &e)
        {
            TRACE("Package transaction failed, try again: %s", e.what());
        }
    }

    if (!transaction_complete)
    {
        //TRACE_ERROR_TIMEOUT(4.0, "Cannot transmit packages to the IO board.");
        throw runtime_error("Could not transmit the package after " + to_string(maxRetries) + " retries.");
    }

    TRACE("<");
    return package.data;
}

IoBoardCommunication::Package IoBoardCommunication::transceive_package(Package send_package)
{
    TRACE(">");

    transmit_package(send_package);
    Package return_package = receive_package();

    if (send_package.command != return_package.command)
    {
        throw(runtime_error(
                "The returned command (" + to_string(return_package.command) +
                ") does not contain the same error message as the requested command (" + to_string(send_package.command) +
                "."));
    }

    TRACE("<");
    return return_package;
}

void IoBoardCommunication::transmit_package(Package package)
{
    TRACE(">");

    vector<unsigned char> buffer = vector<unsigned char>(Data().size() + 3);
    buffer[0] = sof;
    buffer[1] = package.command;
    copy(package.data.begin(), package.data.end(), buffer.begin() + 2);
    buffer.back() = calculate_checksum(buffer);

    serial.writePort(buffer);

    TRACE("<");
}

IoBoardCommunication::Package IoBoardCommunication::receive_package()
{
    TRACE(">");

    bool packageFound = false;
    vector<unsigned char> data;

    while (!packageFound)
    {
        vector<unsigned char> serialData = serial.readPort(Data().size() + 3 - data.size());

        if (serialData.size() == 0)
        {
            throw(runtime_error("Did not receive any character"));
        }

        data.insert(data.end(), serialData.begin(), serialData.end());

        if (data.size() >= (Data().size() + 3))
        {
            if (data[0] == sof)
            {
                // Analyze if we have a correct package.
                if (calculate_checksum(data) == 0x00)
                {
                    // If you add all values including the checksum byte, the inverted sum of all values should be 0x00.
                    packageFound = true;
                }
            }

            if (!packageFound)
            {
                // Either the first character of the data stream was not a SOF,
                // or the first character is a SOF but the checksum doesn't match.
                // In both cases, search for a new SOF.
                auto sofIndex = find(data.begin() + 1, data.end(), sof);

                if (sofIndex != data.end())
                {
                    // Set data equal to the last part of the vector.
                    data = vector<unsigned char>(sofIndex, data.end());
                }
                else
                {
                    data = vector<unsigned char>();
                }
            }
        }
    }

    Package package;
    package.command = data[1];
    copy(data.begin() + 2, data.end() - 1, package.data.begin());

    TRACE("<");
    return package;
}

unsigned char IoBoardCommunication::calculate_checksum(vector<unsigned char> buffer)
{
    TRACE(">");

    unsigned char checksum = 0x00;

    for (auto data = buffer.begin() + 1; data != buffer.end(); data++)
    {
        checksum += *data;
    }

    TRACE("<");
    return ~checksum;
}
