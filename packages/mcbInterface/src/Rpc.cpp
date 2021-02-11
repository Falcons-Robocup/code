// Copyright 2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#include <iostream>

#include "Rpc.hpp"

using std::exception;
using std::string;
using std::runtime_error;
using std::to_string;

namespace Rpc
{
Client::Client(std::string host, uint16_t port) :
    context(1), socket(context, ZMQ_REQ)
{
    address = "tcp://" + host + string(":") + to_string(port);
    initialize();
}

Client::~Client()
{
    terminate();
}

void Client::initialize()
{
    socket = zmq::socket_t(context, ZMQ_REQ);
    socket.setsockopt<int>(ZMQ_RCVTIMEO, 1000);
    socket.connect(address);
}

void Client::terminate()
{
    socket.close();
}

void Client::call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response)
{
    try {
        zmq::message_t method_message(1);
        zmq::message_t request_message(request.ByteSize());

        memcpy(method_message.data(), (void*) &method, sizeof(method));
        request.SerializeToArray(request_message.data(), request_message.size());

        socket.send(method_message, ZMQ_SNDMORE);
        socket.send(request_message);

        zmq::message_t result_message;
        zmq::message_t response_message;

        socket.recv(&result_message);
        socket.recv(&response_message);

        uint8_t result = *((uint8_t *) result_message.data());
        if (result == 0) {
            response.ParseFromArray(response_message.data(), response_message.size());
        }
        else {
            throw(runtime_error(string((const char *) response_message.data(), response_message.size())));
        }
    }
    catch (exception& e) {
        terminate();
        initialize();

        throw(e);
    }
}

Server::Server(uint16_t port) :
    context(1), socket(context, ZMQ_REP)
{
    socket.bind((string("tcp://*:") + to_string(port)).c_str());
}

Server::~Server()
{

}

void Server::run()
{
    while (true) {
        zmq::message_t method_message;
        zmq::message_t request_message;
        socket.recv(&method_message);
        socket.recv(&request_message);

        uint8_t method = *((uint8_t *) method_message.data());

        uint8_t result;
        zmq::message_t response_message;
        try
        {
            ::google::protobuf::Message* request = get_request_prototype(method);
            ::google::protobuf::Message* response = get_response_prototype(method);

            request->ParseFromArray(request_message.data(), request_message.size());

            call(method, *request, *response);

            response_message.rebuild(response->ByteSize());
            response->SerializeToArray(response_message.data(), response_message.size());

            result = 0;

            delete request;
            delete response;
        }
        catch (std::exception& e) {
            result = 1;
            response_message.rebuild(strlen(e.what()));
            memcpy(response_message.data(), (void*) e.what(), strlen(e.what()));
        }

        zmq::message_t result_message(1);
        memcpy(result_message.data(), (void*) &result, sizeof(result));
        socket.send(result_message, ZMQ_SNDMORE);
        socket.send(response_message);
    }
}
}
