 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
