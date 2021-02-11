// Copyright 2016 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPacketAbstract.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef CPACKETABSTRACT_HPP_
#define CPACKETABSTRACT_HPP_

#include <stddef.h>

class cPacketAbstract
{
    public:
        cPacketAbstract() {}
        virtual ~cPacketAbstract() {}

        virtual size_t getSize() { return 0; }
        virtual void getSerialized(uint8_t* packet, size_t &packetSize) {}
};

#endif /* CPACKETABSTRACT_HPP_ */
