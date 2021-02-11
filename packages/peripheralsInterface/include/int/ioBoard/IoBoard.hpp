// Copyright 2017-2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IoBoard.hpp
 *
 *    Created on: Jul 23, 2017
 *    Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_IOBOARD_IOBOARD_HPP_
#define INCLUDE_INT_IOBOARD_IOBOARD_HPP_

#include <mutex>
#include <string>

#include "int/ioBoard/IoBoardCommunication.hpp"

class IoBoard {
public:
    enum GetCommand {
        STATUS,
        VERSION,
    };

    enum SetCommand {
        SHOOT,
        HOME,
        LEVER_SPEED,
        HEIGHT,
        BOOTLOADER,
        KEEPER_FRAME_RIGHT,
        KEEPER_FRAME_UP,
        KEEPER_FRAME_LEFT
    };

    struct Status {
        bool inPlay;
        bool softwareOn;
    };

    IoBoard(std::string portName);

    void initialize();

    Status getStatus();
    void setShoot(unsigned char shootPower);
    void setHome();
    void setHeight(unsigned char height);
    void setLeverSpeed(unsigned char speed);
    void setKeeperFrameRight();
    void setKeeperFrameUp();
    void setKeeperFrameLeft();

private:
    bool isFirmwareVersionCorrect();
    void upgradeFirmware(std::string portName);

    std::mutex communicationBusy;

    IoBoardCommunication * communication;
    string portName;

    Status status;
    Status measuredStatus;
};

#endif /* INCLUDE_INT_IOBOARD_IOBOARD_HPP_ */
