// Copyright 2019-2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Rpc.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: Edwin Schreuder
 */
#include <stdexcept>

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
    socket.connect(string("tcp://" + host + string(":") + to_string(port)).c_str());

    socket.setsockopt(ZMQ_REQ_CORRELATE, 1);
    socket.setsockopt(ZMQ_REQ_RELAXED, 1);
    socket.setsockopt(ZMQ_RCVTIMEO, 1000); //timeout after 1000ms
    socket.setsockopt(ZMQ_SNDTIMEO, 1000); //timeout after 1000ms
}

Client::~Client()
{

}

void Client::call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response)
{
    zmq::message_t method_message(1);
    zmq::message_t request_message(request.ByteSizeLong());

    memcpy(method_message.data(), (void*) &method, sizeof(method));
    request.SerializeToArray(request_message.data(), request_message.size());

    socket.send(method_message, zmq::send_flags::sndmore);
    socket.send(request_message, zmq::send_flags::none);

    zmq::message_t result_message;
    zmq::message_t response_message;

    zmq::recv_result_t resu_result = socket.recv(result_message);
    zmq::recv_result_t resp_result = socket.recv(response_message);

    uint8_t result = *((uint8_t *) result_message.data());
    if (result == 0 && resu_result && resp_result) {
        response.ParseFromArray(response_message.data(), response_message.size());
    }
    else {
        throw(runtime_error(string((const char *) response_message.data(), response_message.size())));
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
        zmq::recv_result_t method_result = socket.recv(method_message);
        zmq::recv_result_t request_result = socket.recv(request_message);

        uint8_t method = *((uint8_t *) method_message.data());

        uint8_t result;
        zmq::message_t response_message;
        if (method_result && request_result)
        {
            try
            {
                ::google::protobuf::Message* request = get_request_prototype(method);
                ::google::protobuf::Message* response = get_response_prototype(method);

                request->ParseFromArray(request_message.data(), request_message.size());

                call(method, *request, *response);

                response_message.rebuild(response->ByteSizeLong());
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
        }
        else
        {
            result = 1;
            const char* msg = "received method or request was empty";
            response_message.rebuild(strlen(msg));
            memcpy(response_message.data(), (void*) msg, strlen(msg));
        }

        zmq::message_t result_message(1);
        memcpy(result_message.data(), (void*) &result, sizeof(result));
        socket.send(result_message, zmq::send_flags::sndmore);
        socket.send(response_message, zmq::send_flags::none);
    }
}
}
