// Copyright 2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
// THIS FILE IS GENERATED, DO NOT EDIT!

#ifndef KICKER_
#define KICKER_

#include <string>

#include "Rpc.hpp"
#include "Kicker.pb.h"

namespace Kicker
{
class Client : public Rpc::Client
{
public:
    Client(const std::string host, uint16_t port);
    ~Client();

    ::kicker::Empty shoot(const ::kicker::ShootPower& request);
    ::kicker::Empty set_height(const ::kicker::Height& request);
};

class Server : public Rpc::Server
{
public:
    Server(uint16_t port);
    ~Server();

protected:
    virtual void shoot(const ::kicker::ShootPower& request, ::kicker::Empty& response) = 0;
    virtual void set_height(const ::kicker::Height& request, ::kicker::Empty& response) = 0;

private:
    ::google::protobuf::Message* get_request_prototype(uint8_t method);
    ::google::protobuf::Message* get_response_prototype(uint8_t method);
    void call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response);
};
}

#endif // KICKER_
