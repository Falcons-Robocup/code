// THIS FILE IS GENERATED, DO NOT EDIT!
#include "int/Kicker.rpc.pb.h"

namespace Kicker
{
Client::Client(const std::string host, uint16_t port) :
    Rpc::Client(host, port)
{
}

Client::~Client()
{
}

::kicker::Empty Client::shoot(const ::kicker::ShootPower& request)
{
    ::kicker::Empty response;
    call(0, request, response);
    return response;
}

::kicker::Empty Client::set_height(const ::kicker::Height& request)
{
    ::kicker::Empty response;
    call(1, request, response);
    return response;
}

Server::Server(uint16_t port) :
    Rpc::Server(port)
{
}

Server::~Server()
{
}

::google::protobuf::Message* Server::get_request_prototype(uint8_t method)
{
    ::google::protobuf::Message* request_prototype;
    switch (method)
    {
    case 0:
        request_prototype = new ::kicker::ShootPower();
        break;
    case 1:
        request_prototype = new ::kicker::Height();
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }

    return request_prototype;
}

::google::protobuf::Message* Server::get_response_prototype(uint8_t method)
{
    ::google::protobuf::Message* response_prototype;
    switch (method)
    {
    case 0:
        response_prototype = new ::kicker::Empty();
        break;
    case 1:
        response_prototype = new ::kicker::Empty();
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }

    return response_prototype;
}

void Server::call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response)
{
    switch (method)
    {
    case 0:
        shoot(::google::protobuf::down_cast<const ::kicker::ShootPower&>(request), ::google::protobuf::down_cast<::kicker::Empty&>(response));
        break;
    case 1:
        set_height(::google::protobuf::down_cast<const ::kicker::Height&>(request), ::google::protobuf::down_cast<::kicker::Empty&>(response));
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }
}
}
