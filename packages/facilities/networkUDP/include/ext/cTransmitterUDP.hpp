 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
