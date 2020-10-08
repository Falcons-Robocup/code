 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Rpc.hpp
 *
 *  Created on: Apr 18, 2019
 *      Author: Edwin Schreuder
 */

#ifndef RPC_RPC_HPP_
#define RPC_RPC_HPP_

#include <string>

#include <google/protobuf/message.h>

#include <zmq.hpp>

namespace Rpc
{
class Client
{
public:
    Client(std::string host, uint16_t port);
    ~Client();

protected:
    void call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response);

private:
    zmq::context_t context;
    zmq::socket_t socket;

    std::string address;

    void initialize();
    void terminate();
};

class Server
{
public:
    Server(uint16_t port);
    virtual ~Server();

    void run();

protected:
    virtual ::google::protobuf::Message* get_request_prototype(uint8_t method) = 0;
    virtual ::google::protobuf::Message* get_response_prototype(uint8_t method) = 0;
    virtual void call(
        uint8_t method,
        const ::google::protobuf::Message& request,
        ::google::protobuf::Message& response) = 0;

    void initialize();
    void terminate();

private:
    zmq::context_t context;
    zmq::socket_t socket;
};
}

#endif /* RPC_RPC_HPP_ */
