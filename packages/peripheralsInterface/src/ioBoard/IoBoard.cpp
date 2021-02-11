// Copyright 2017-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IoBoard.cpp
 *
 *    Created on: Jul 18, 2017
 *            Author: Edwin Schreuder
 */

#include <exception>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <cDiagnostics.hpp>
#include <falconsCommon.hpp>
#include "tracing.hpp"

#include "int/ioBoard/IoBoardCommunication.hpp"
#include "int/ioBoard/IoBoard.hpp"

using std::cout;
using std::endl;
using std::map;
using std::mutex;
using std::string;
using std::unique_lock;
using std::vector;

const string IoBoardFirmwareVersion = "00000003";

map<IoBoard::SetCommand, unsigned char> IoBoardSetCommandValues = {
    {IoBoard::SHOOT, 0x02},
    {IoBoard::HOME, 0x05},
    {IoBoard::LEVER_SPEED, 0x06},
    {IoBoard::HEIGHT, 0x07},
    {IoBoard::BOOTLOADER, 0xAA},
    {IoBoard::KEEPER_FRAME_RIGHT, 0x09},
    {IoBoard::KEEPER_FRAME_UP, 0x0A},
    {IoBoard::KEEPER_FRAME_LEFT, 0x0B}

};

map<IoBoard::GetCommand, unsigned char> IoBoardGetCommandValues = {
    {IoBoard::STATUS, 0x08},
    {IoBoard::VERSION, 0x56},
};

IoBoard::IoBoard(string portName) :
            portName(portName)
{
    TRACE(">");

    communication = new IoBoardCommunication(portName);

    status = {false};
    measuredStatus = {false};

    TRACE("<");
}

void IoBoard::initialize()
{
    TRACE(">");

    if (!isFirmwareVersionCorrect())
    {
        upgradeFirmware(portName);

        if (!isFirmwareVersionCorrect())
        {
            throw(runtime_error("Firmware version still not correct after upgrade."));
        }
    }

    TRACE("<");
}

bool IoBoard::isFirmwareVersionCorrect()
{
    TRACE(">");

    bool firmwareVersionCorrect = false;

    IoBoardCommunication::Data data = communication->get(IoBoardGetCommandValues[VERSION]);

    string firmwareVersion(data.begin(), data.end());

    if (IoBoardFirmwareVersion != firmwareVersion)
    {
        cout << "Firmware version of the IO board is incorrect: " << firmwareVersion << " instead of " << IoBoardFirmwareVersion << "." << endl;
        TRACE_ERROR("Firmware version of the IO board is incorrect: %s", firmwareVersion.c_str());
    }
    else
    {
        cout << "IO board firmware version: " << firmwareVersion << "." << endl;
        firmwareVersionCorrect = true;
    }

    TRACE("<");
    return firmwareVersionCorrect;
}

void IoBoard::upgradeFirmware(string portName)
{

    //    try {
    //        cout << "Setting IO board to bootloader mode." << endl;
    //        TRACE_INFO("Setting IO board to bootloader mode.");
    //    }
    //    catch (exception &e) {
    //        throw(e);
    //    }
    //
    //    delete communication;
    //
    //    cout << "IO board firmware upgrade started." << endl;
    //    TRACE_INFO("Upgrading firmware to version %s.", IoBoardFirmwareVersion.c_str());
    //
    //    if (system(string("avrdude -p atxmega64a3 -P " + portName + "-c avr109 -b 115200 -U flash:w:../scripts/IoBoard.hex").c_str()) != 0) {
    //        cerr << "Failed to upgrade the IO board firmware." << endl;
    //        TRACE_ERROR("Failed to upgrade the IoBoard firmware.");
    //        throw(runtime_error("Failed to upgrade the IoBoard firmware."));
    //    }
    //
    //    communication = new IoBoardCommunication(portName);
    //
    //    cout << "IO board firmware upgrade finished." << endl;
    //    TRACE_INFO("IO board firmware upgrade finished.");
}

IoBoard::Status IoBoard::getStatus()
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    IoBoardCommunication::Data data = communication->get(IoBoardGetCommandValues[STATUS]);

    Status newMeasuredStatus;
    newMeasuredStatus.inPlay = data[0] != 0;
    newMeasuredStatus.softwareOn = data[1] != 0;

    if (newMeasuredStatus.inPlay == measuredStatus.inPlay)
    {
        // Only update the actual status when it has not changed since the previous cycle.
        status.inPlay = newMeasuredStatus.inPlay;
    }
    if (newMeasuredStatus.softwareOn == measuredStatus.softwareOn)
    {
        // Only update the actual status when it has not changed since the previous cycle.
        status.softwareOn = newMeasuredStatus.softwareOn;
    }
    measuredStatus = newMeasuredStatus;

    TRACE("<");
    return status;
}

void IoBoard::setShoot(unsigned char shootPower)
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    IoBoardCommunication::Data data;
    data[0] = shootPower;

    communication->set(IoBoardSetCommandValues[SHOOT], data);

    TRACE("<");
}

void IoBoard::setHome()
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[HOME], IoBoardCommunication::Data());

    TRACE("<");
}

void IoBoard::setHeight(unsigned char height)
{
    TRACE(">");

    IoBoardCommunication::Data data;
    data[0] = height;

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[HEIGHT], data);

    TRACE("<");
}

void IoBoard::setLeverSpeed(unsigned char speed)
{
    TRACE(">");

    IoBoardCommunication::Data data;
    data[0] = speed;

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[LEVER_SPEED], data);

    TRACE("<");
}

void IoBoard::setKeeperFrameRight()
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[KEEPER_FRAME_RIGHT], IoBoardCommunication::Data());

    TRACE("<");
}

void IoBoard::setKeeperFrameUp()
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[KEEPER_FRAME_UP], IoBoardCommunication::Data());

    TRACE("<");
}

void IoBoard::setKeeperFrameLeft()
{
    TRACE(">");

    unique_lock<mutex> lock(communicationBusy);

    communication->set(IoBoardSetCommandValues[KEEPER_FRAME_LEFT], IoBoardCommunication::Data());

    TRACE("<");
}
