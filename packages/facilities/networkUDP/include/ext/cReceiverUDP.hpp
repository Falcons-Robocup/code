// Copyright 2015 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#ifndef CRECEIVERUDP_HPP_
#define CRECEIVERUDP_HPP_

#include <string>
#include <vector>
#include <netinet/in.h>
#include <pthread.h>

#include "cAbstractObserverByteArray.hpp"

namespace Facilities
{
namespace Network
{

class cReceiverUDP
{
    public:
    cReceiverUDP(
            const std::string multicastAddress,
            const unsigned int portNumber);
    virtual ~cReceiverUDP();

    bool isSocketOpen();
    void attachObserver(cAbstractObserverByteArray *observer);
    void detachObserver(cAbstractObserverByteArray *observer);

private:
        struct sockaddr_in _localAddress;
        struct ip_mreq _multicastGroup;
        bool _socketOpened;
        int _socketFileDescr;
        pthread_t _receiveThread;
        std::vector<cAbstractObserverByteArray *> _observerList;

        void openSocket();
        void closeSocket();
        static void* notifyNewPacket(void *context);

};


} /* namespace Network */
} /* namespace Facilities */

#endif /* CRECEIVERUDP_HPP_ */
