// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
