 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef _INCLUDED_RTDB2_COMM2_MULTICASTSOCKET_HPP_
#define _INCLUDED_RTDB2_COMM2_MULTICASTSOCKET_HPP_

// JFEI Dec. 2018: based on MulticastSocket from comm v1, by Cambada

#include <arpa/inet.h>
#include <string>

class MulticastSocket
{
  public:
    MulticastSocket();
    ~MulticastSocket();

    int openSocket(std::string const &interface, std::string const &group, int port, bool loopback = false);
    int closeSocket();
    int sendData(void* data, int size);
    int receiveData(void* data, int size);

    int getSocket() const { return _socket; }
    std::string getInterface() const { return _interfaceName; }
    std::string getIpAddress() const { return _address; }

  private:
    int         _port;
    int         _socket;
    int         _interfaceIndex;
    bool        _loopback;
    std::string _interfaceName;
    std::string _address;
    std::string _group;

    struct sockaddr_in _destAddress;
    bool resolve();
    bool autoSelectInterface();
};

#endif
