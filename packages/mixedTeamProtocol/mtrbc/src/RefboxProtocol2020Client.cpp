// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <exception>
#include <stdexcept>

#include <boost/thread.hpp>
#include "RefboxCommand.h"
#include "RefboxProtocol2020Client.h"
#include <iostream>
#include <netdb.h>
#include <unistd.h>     /* for close() */
#include <sys/types.h>  /* for type definitions */
#include <sys/socket.h> /* for socket API calls */
#include <netinet/in.h> /* for address structs */
#include <arpa/inet.h>  /* for sockaddr_in */

#define MAX_LEN 4096 /* maximum receive string size */

using namespace std;

namespace rbc
{
    /****************************************
     * Constructor
     ***************************************/
    RefboxProtocol2020Client::RefboxProtocol2020Client() : sockfd(-1)
    {
    }

    /****************************************
     * Destructor
     ***************************************/
    RefboxProtocol2020Client::~RefboxProtocol2020Client()
    {
        close(sockfd);
    }

    /****************************************
     * connect()
     ***************************************/
    void RefboxProtocol2020Client::connect(const string &host, int port)
    {
        try {
            struct hostent *serveraddr;

            // init socket
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            cout << "RefboxProtocol2020_Client (Connect2Host): sockfd = '" << sockfd << "'" << endl;

            // setting host:
            cout << "RefboxProtocol2020_Client (Connect2Host): setting host '" << host << "'." << endl;
            address.sin_family = AF_INET;
            if ((serveraddr = gethostbyname(host.c_str())) == 0)
            {
                cout << "RefboxProtocol2020_Client (Connect2Host): ### unknown host!" << endl;
                throw(std::runtime_error("RefboxProtocol2020_Client (Connect2Host): ### unknown host!"));
            }

            memcpy(&address.sin_addr, serveraddr->h_addr, serveraddr->h_length);
            cout << "RefboxProtocol2020_Client (Connect2Host): setting port '" << port << "'." << endl;
            address.sin_port = htons(port);
            len = sizeof(address);

            cout << "RefboxProtocol2020_Client (Connect2Host): trying to connect socket ..." << endl;
            if (::connect(sockfd, (struct sockaddr *)&address, len) != 0)
            {
                perror("Could not connect to server");
                cout << "RefboxProtocol2020_Client (Connect2Host): ### couldn't connect to server!" << endl;
                throw(std::runtime_error("RefboxProtocol2020_Client (Connect2Host): ### couldn't connect to server!"));
            }

            len = sizeof(address);

            cout << "RefboxProtocol2020_Client (Connect2Host): Connected!" << endl;
            cout << "RefboxProtocol2020_Client (Connect2Host): IP_Address: '" << inet_ntoa(address.sin_addr) << "'" << endl;
            cout << "RefboxProtocol2020_Client (Connect2Host): Port: '" << ntohs(address.sin_port) << "'" << endl;
        }
        catch (std::exception &e)
        {
            if (sockfd != -1)
            {
                ::close(sockfd);
                sockfd = -1;
            }
            throw(e);
        }
    }

    /****************************************
     * disconnect()
     ***************************************/
    void RefboxProtocol2020Client::disconnect()
    {
        cout << "RefboxProtocol2020_Client (DisConnect): Closing socket ..." << endl;
        close(sockfd);

        cout << "RefboxProtocol2020_Client (DisConnect): reinit adress ..." << endl;
        // reinit adress
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr("127.0.0.1");
        address.sin_port = htons(0);
        // set adress-length
        len = sizeof(address);

        cout << "RefboxProtocol2020_Client (DisConnect): Disconnected." << endl;
    }

    /****************************************
     * listen()
     ***************************************/
    int RefboxProtocol2020Client::listen(RefBoxCallback *callbackClass)
    {
        char recv_str[MAX_LEN + 1];   /* buffer to receive string */
        int recv_len;                 /* length of string received */
        struct sockaddr_in from_addr; /* packet source */
        unsigned int from_len;        /* source addr length */

        int result = -2;

        try
        {
            do
            {
                // clear the receive buffers & structs
                memset(recv_str, 0, sizeof(recv_str));
                from_len = sizeof(from_addr);
                memset(&from_addr, 0, from_len);

                // block waiting to receive a packet
                if ((recv_len = recvfrom(sockfd, recv_str, MAX_LEN, 0, (struct sockaddr *)&from_addr, &from_len)) < 0)
                {
                    perror("recvfrom() failed");
                    continue;
                }

                // output received string
                std::cout << "Received " << recv_len << " bytes from: " << inet_ntoa(from_addr.sin_addr) << std::endl
                          << std::flush;
                std::string jsonString = recv_str;
                std::cout << "string received: " << jsonString << std::endl
                          << std::flush;

                RefboxCommand command = RefboxCommand::fromString(jsonString);
                if (command.isValid())
                {
                    callbackClass->handleCommand(command);
                }

                //cout << "RefboxProtocol2020_Client (Listen): Got command '" << strCommand << "' ('" << rcvd << "') from server. ('" << result << "' bytes received.)" << endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }
            while (recv_len > 0);
        }
        catch (boost::thread_interrupted &)
        {
            cout << "Thread is stopped" << endl;
        }

        return result;
    }
}
