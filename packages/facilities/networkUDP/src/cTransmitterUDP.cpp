// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTransmitterUDP.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#include "ext/cTransmitterUDP.hpp"

#include "tracing.hpp"
#include "falconsCommon.hpp"

#include <string.h>
#include <arpa/inet.h>
#include <exception>
#include <stdexcept>
#include <errno.h>

using namespace Facilities::Network;

using std::exception;
using std::runtime_error;
using std::string;

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
cTransmitterUDP::cTransmitterUDP(
        const string multicastAddress,
        const unsigned int portNumber,
        const bool loopEnabled,
        const unsigned int maxHops)
{
    try
    {
    	TRACE("constructing with multicastAddress=%s port=%d loopEnabled=%d, maxHops=%d", multicastAddress.c_str(), portNumber, loopEnabled, maxHops);
        /* Initialize default values */
        _socketFileDescr = -1;
        _socketOpened = false;
        _sockOptLoop = 0;
        memset((char *) &_localAddress, 0, sizeof(_localAddress));
        memset((char *) &_groupAddress, 0, sizeof(_groupAddress));

        /* Set max Time To Live */
        _ttl = maxHops;

        /* Fill group socket address */
        _groupAddress.sin_family = AF_INET;
        _groupAddress.sin_port = htons(portNumber);
        _groupAddress.sin_addr.s_addr = inet_addr(multicastAddress.data());

        /* Fill local socket address */
        _localAddress.s_addr = INADDR_ANY;

        if(loopEnabled)
        {
            _sockOptLoop = 1;
        }

        /* Open Socket */
    	//TRACE("opening socket");
        openSocket();
    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
        throw e;
    }
}

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
cTransmitterUDP::~cTransmitterUDP()
{
    try
    {
        if (isSocketOpen())
        {
            closeSocket();
        }
    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
    }
}

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
#define BUFSIZE 65536
void cTransmitterUDP::sendPacket(cByteArray *packet)
{
    std::vector<uint8_t> data;
    uint8_t sendBuf[BUFSIZE];
    ssize_t rc;

    try
    {
        if(!isSocketOpen())
        {
            throw runtime_error("Failed to send packet, socket is closed");
        }

        packet->getData(data);
        if(data.size() > BUFSIZE-2)
        {
            throw runtime_error("Buffer size exceeded");
        }
        // copy to buffer
        for (int it = 0; it < (int)data.size(); ++it)
        {
            sendBuf[it] = data[it];
        }
        
        /* Send serialized packet */
        rc = sendto(_socketFileDescr, (const void*)sendBuf, data.size(), 0, (struct sockaddr*)&_groupAddress, sizeof(struct sockaddr_in));

		if (rc==-1)
		{
			TRACE("Failed to send packet, error during sending: %s", strerror(errno));
			throw runtime_error("Failed to send packet, error during sending");
		}
    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
        throw e;
    }
}

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
bool cTransmitterUDP::isSocketOpen()
{
    return _socketOpened;
}

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
void cTransmitterUDP::openSocket()
{
    try
    {
        /* Open socket datagram */
        _socketFileDescr = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if(_socketFileDescr < 0)
        {
            throw runtime_error("Failed to create file descriptor");
        }

        /* Socket is successfully opened */
        _socketOpened = true;

        /* Set own loopback so we (do/don't) receive our own messages */
        if(setsockopt(_socketFileDescr, IPPROTO_IP, IP_MULTICAST_LOOP, &_sockOptLoop, sizeof(_sockOptLoop)) < 0)
        {
            throw runtime_error("Failed to disable loop-package");
        }

        /* Appoint local IP address to multicast UDP socket */
        if(setsockopt(_socketFileDescr, IPPROTO_IP, IP_MULTICAST_IF, &_localAddress, sizeof(in_addr)) < 0)
        {
            throw runtime_error("Failed to couple local IP address to multicast socket");
        }

        /* Set TTL (Time To Live) for defining number of hops */
        if(setsockopt(_socketFileDescr, IPPROTO_IP, IP_MULTICAST_TTL, &_ttl, sizeof(_ttl)) < 0)
        {
            throw runtime_error("Failed to set TTL to multicast socket");
        }

    }
    catch (exception &e)
    {
        /* Close socket */
        closeSocket();

        TRACE("Caught exception: %s", e.what());
        throw e;
    }
}

/*!
 * \brief A single line function description
 *
 * A more detailed explanation
 * can go here...
 *
 * \param[in] arg1 Argument description
 *
 * \retval int Return value description
 *
 * \note Optional note
 */
void cTransmitterUDP::closeSocket()
{
    if(_socketOpened == true)
    {
        shutdown(_socketFileDescr, SHUT_RDWR);
        close(_socketFileDescr);
    }

    _groupAddress.sin_family = AF_INET;
    _groupAddress.sin_addr.s_addr = inet_addr("127.0.1.1");
    _groupAddress.sin_port = htons(0);

    _socketFileDescr = -1;
    _socketOpened = false;
}
