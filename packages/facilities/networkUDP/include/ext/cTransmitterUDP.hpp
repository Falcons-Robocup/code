// Copyright 2015 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * $Id: cTransmitterUDP.hpp 1603 2015-05-18 21:49:43Z jmbm $
 *
 * cTransmitterUDP.hpp
 *
 *  Created on: Feb 26, 2015
 *      Author: Tim Kouters
 */

#ifndef CTRANSMITTERUDP_HPP_
#define CTRANSMITTERUDP_HPP_

#include <string>
#include <netinet/in.h>
#include <stdint.h>

#include "cByteArray.hpp"

namespace Facilities
{
namespace Network
{


class cTransmitterUDP
{
    public:
        cTransmitterUDP(
                const std::string multicastAddress,
                const unsigned int portNumber,
                const bool loopEnabled,
                const unsigned int maxHops);
        ~cTransmitterUDP();

        void sendPacket(cByteArray *packet);
        bool isSocketOpen();

    private:
        struct sockaddr_in _groupAddress;
        struct in_addr _localAddress;
        int  _socketFileDescr;
        bool _socketOpened;
        u_char  _sockOptLoop;
        u_char _ttl;

        void openSocket();
        void closeSocket();
};

} /* namespace Network */
} /* namespace Facilities */

#endif /* CTRANSMITTERUDP_HPP_ */
