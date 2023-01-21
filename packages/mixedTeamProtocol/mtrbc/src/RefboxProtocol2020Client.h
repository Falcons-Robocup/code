// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef REFBOX_PROTOCOL2020_CLIENT_H_
#define REFBOX_PROTOCOL2020_CLIENT_H_

#include <string.h>
#include <netinet/in.h>

namespace rbc
{
    class RefBoxCallback
    {
    public:
        virtual void handleCommand(RefboxCommand &command) = 0;
    };

    class RefboxProtocol2020Client
    {
    public:
        RefboxProtocol2020Client();
        ~RefboxProtocol2020Client();
        void connect(const std::string &host, int port);
        void disconnect();
        int listen(RefBoxCallback *callbackClass);

    private:
        int sockfd;
        int len;
        struct sockaddr_in address;
    };
}

#endif
