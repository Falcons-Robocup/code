 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cReceiverUDP.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#include "ext/cReceiverUDP.hpp"

#include "tracing.hpp"
#include "falconsCommon.hpp"

#include <string.h>
#include <arpa/inet.h>
#include <exception>
#include <stdexcept>

using std::exception;
using std::runtime_error;
using std::string;

using namespace Facilities::Network;

/*!
 * \brief Standardized UDP receiver constructor for receiving serialized bytes
 *
 * cReceiverUDP class is a standardization of receiving a byte array over UDP packages
 * The class listens to a certain address and portnumber.
 *
 * \param[in] multicastAddress  IP number in the multicast range
 * \param[in] portNumber        Port number to receive UDP packages from
 *
 *
 * \note The address given to this constructor needs to be in the multicast range
 */
cReceiverUDP::cReceiverUDP(
        const string multicastAddress,
        const unsigned int portNumber)
{
    try
    {
    	char localIP[] = "255.255.255.255";
    	GetPrimaryIp(localIP, sizeof(localIP));
    	string localAddress = string(localIP);

    	TRACE("constructing with localAddress=%s multicastAddress=%s port=%d", localAddress.c_str(), multicastAddress.c_str(), portNumber);
        /* Initialize default values */
        _socketFileDescr = -1;
        _socketOpened = false;
        memset((char *) &_localAddress, 0, sizeof(_localAddress));
        memset((char *) &_multicastGroup, 0, sizeof(_multicastGroup));

        /* Fill local socket address (use ANY IP-address) */
        _localAddress.sin_family = AF_INET;
        _localAddress.sin_port = htons(portNumber);
        _localAddress.sin_addr.s_addr = INADDR_ANY;

        /* Fill in multicast address to listen to */
        _multicastGroup.imr_multiaddr.s_addr = inet_addr(multicastAddress.data());
        _multicastGroup.imr_interface.s_addr = inet_addr(localAddress.data());

        /* Open Socket */
        openSocket();

        /* Start receiving thread */
        if(pthread_create(&_receiveThread, NULL, Facilities::Network::cReceiverUDP::notifyNewPacket, this))
        {
            throw runtime_error("Failed to create receiving thread");
        }

    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
        throw e;
    }
}

/*!
 * \brief Destructor of cReceiverUDP class
 *
 */
cReceiverUDP::~cReceiverUDP()
{
    try
    {
    	pthread_cancel(_receiveThread);

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
 * \brief Callback function called when receiving a new UDP packet
 *
 * This function is called by the UDP receiving thread to fetch the
 * UDP data packet and send it around to every observer subscribed
 * to this UDP receiver class
 *
 * \param[in] *context  Thread context with pointer to original class
 *
 * \retval void* return code of the thread
 *
 */
void* cReceiverUDP::notifyNewPacket(void *context)
{
    bool loop = true;
    uint8_t recBuf[65536] = {0,};
    int readSize = 0;
    cReceiverUDP *ctx = (cReceiverUDP *)context;
    cByteArray byteData;
   
    try
    {
        while (loop)
        {
            if (!ctx->isSocketOpen())
            {
                loop = false;
                throw runtime_error("Failed to receive packet, socket is closed");
            }

            /* Read message from UDP socket */
            readSize = read(ctx->_socketFileDescr, &recBuf, sizeof(recBuf));

            if(readSize <= 0)
            {
                // EINTR == "interrupted system call" -> Can happen, is not fatal.
                if (errno == EINTR)
                {
                    TRACE("Failed to read package: %s. Retrying...", strerror(errno));
                }
                else
                {
                    TRACE("Failed to read package: %s. Closing...", strerror(errno));
                    loop = false;
                }

                continue;
            }

            // convert to vector and store in byteData
            std::vector<uint8_t> data(recBuf, recBuf+readSize);
            byteData.setData(data);

            std::cout << "data.size(): " << data.size() << std::endl;

            /* Pass around the byte array to all subscribers */
            for(size_t i = 0; i < ctx->_observerList.size(); i++)
            {
            	ctx->_observerList.at(i)->notifyNewPacket(byteData);
            }

        } /* End while loop */
    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
        throw e;
    }

    return 0;
}

/*!
 * \brief Verify whether the socket is opened
 *
 * \retval bool Returns TRUE when socket is opened
 *
 */
bool cReceiverUDP::isSocketOpen()
{
    return _socketOpened;
}

/*!
 * \brief Attach a byte array observer
 *
 * This function is used to attach an observer class of type
 * cAbstractObserverByteArray.
 * When an observer is attached it's added to the observer vector
 *
 * \param[in] observer Pointer to an observer class
 *
 */
void cReceiverUDP::attachObserver(cAbstractObserverByteArray *observer)
{
	_observerList.push_back(observer);
}

/*!
 * \brief Detach a byte array observer
 *
 * This function is used to detach an observer class of type
 * cAbstractObserverByteArray.
 * When an observer is detached it's removed from the observer vector
 *
 * \param[in] observer Pointer to an observer class
 *
 * \note When observer is not in the list, it's ignored and no error is generated
 */
void cReceiverUDP::detachObserver(cAbstractObserverByteArray *observer)
{
	std::vector<cAbstractObserverByteArray *>::iterator iter;

	iter = find(_observerList.begin(), _observerList.end(), observer);

	if (iter != _observerList.end())
	{
		_observerList.erase(iter);
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
void cReceiverUDP::openSocket()
{
    unsigned int socketReuse = 1; // (1): enable reuse

    try
    {
        /* Open socket datagram */
        _socketFileDescr = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if(_socketFileDescr < 0)
        {
            throw runtime_error("Failed to create file descriptor");
        }

        _socketOpened = true;

        /* Enable reuse of the socket for performance reasons */
        if(setsockopt(_socketFileDescr, SOL_SOCKET, SO_REUSEADDR, &socketReuse, sizeof(socketReuse)) < 0)
        {
            throw runtime_error("Failed to enable reuse of socket");
        }

        /* Bind the local socket */
        if (bind(_socketFileDescr, (struct sockaddr *) &_localAddress, sizeof(_localAddress)) < 0)
        {
            throw runtime_error("Failed to bind file descriptor to socket");
        }

        /* Join the multicast group */
        if (setsockopt(_socketFileDescr, IPPROTO_IP, IP_ADD_MEMBERSHIP, &_multicastGroup, sizeof(_multicastGroup)) < 0)
        {
            throw runtime_error("Failed to join the multicast group");
        }

    }
    catch (exception &e)
    {
        TRACE("Caught exception: %s", e.what());
        closeSocket();

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
void cReceiverUDP::closeSocket()
{
    if(_socketOpened == true)
    {
        shutdown(_socketFileDescr, SHUT_RDWR);
        close(_socketFileDescr);
    }

    _localAddress.sin_family = AF_INET;
    _localAddress.sin_addr.s_addr = inet_addr("127.0.1.1");
    _localAddress.sin_port = htons(0);

    _socketFileDescr = -1;
    _socketOpened = false;
}
