// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "MulticastSocket.h"

#include <set>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>
#include <string>

MulticastSocket::MulticastSocket()
{
    _socket = -1;
}

MulticastSocket::~MulticastSocket()
{
    if (_socket != -1) closeSocket();
}

bool MulticastSocket::resolve()
{
    // given interface name _interface (which may be 'auto'), 
    // determine target interface (e.g. wlo), updating _interface
    // its index and target IP address
    // return true if all went well
    
    if (_interfaceName == "auto")
    {
        autoSelectInterface();
        fprintf(stdout, "MulticastSocket::resolve(interface = \"auto\" -> \"%s\")\n", _interfaceName.c_str());
    }
    else
    {
        fprintf(stdout, "MulticastSocket::resolve(interface = \"%s\")\n", _interfaceName.c_str());
    }

    int fd;
    struct ifreq if_info;
    int if_index;

    memset(&if_info, 0, sizeof(if_info));
    strncpy(if_info.ifr_name, _interfaceName.c_str(), IFNAMSIZ-1);

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        fprintf(stderr, "Failed to initialize socket\n");
        return false;
    }
    if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
    {
        fprintf(stderr, "Failed ioctl part 1\n");
        return false;
    }
    if_index = if_info.ifr_ifindex;

    if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
    {
        fprintf(stderr, "Failed ioctl part 2\n");
        return false;
    }
    
    close(fd);
    char buffer[32];
    sprintf(buffer, "%d.%d.%d.%d",
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4],
        (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]);

    // store outputs
    _address = buffer;
    _interfaceName = if_info.ifr_name;
    _interfaceIndex = if_index;

    return true;
}

int MulticastSocket::openSocket(std::string const &interface, std::string const &group, int port, bool loopback)
{
    // if this is a reopen, close previous opening
    if (_socket != -1) closeSocket();

    // store arguments
    _interfaceName = interface;
    _group = group;
    _port = port;
    _loopback = loopback;
    
    // open the socket
    struct sockaddr_in multicastAddress;
    struct ip_mreqn mreqn;
    struct ip_mreq mreq;
    int opt;

    bzero(&multicastAddress, sizeof(struct sockaddr_in));
    multicastAddress.sin_family = AF_INET;
    multicastAddress.sin_port = htons(_port);
    multicastAddress.sin_addr.s_addr = INADDR_ANY;

    bzero(&_destAddress, sizeof(struct sockaddr_in));
    _destAddress.sin_family = AF_INET;
    _destAddress.sin_port = htons(_port);
    _destAddress.sin_addr.s_addr = inet_addr(_group.c_str());

    if ((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        fprintf(stderr, "socket() call failed\n");
        return -1;
    }

    // resolve interface name and index
    resolve();
    memset((void *) &mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex = _interfaceIndex;
    if ((setsockopt(_socket, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
    {
        fprintf(stderr, "setsockopt() call 1 failed\n");
        return -1;
    }

    opt = 1;
    if ((setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
        fprintf(stderr, "setsockopt() call 2 failed\n");
        return -1;
    }
 
    memset((void *) &mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(_group.c_str());
    mreq.imr_interface.s_addr = inet_addr(_address.c_str());

    if ((setsockopt(_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
    {
        fprintf(stderr, "setsockopt() call 3 failed\n");
        return -1;
    }
                        
    // Enable/Disable reception of our own multicast
    opt = _loopback;
    if ((setsockopt(_socket, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
    {
        fprintf(stderr, "setsockopt() call 4 failed\n");
        return -1;
    }

    if (bind(_socket, (struct sockaddr *) &multicastAddress, sizeof(struct sockaddr_in)) == -1)
    {
        fprintf(stderr, "bind() call failed\n");
        return -1;
    }

    return 0;
}

int MulticastSocket::closeSocket()
{
    if (_socket == -1) return 0;

    int ret = shutdown(_socket, SHUT_RDWR);

    _socket = -1;
    return ret;
}

int MulticastSocket::sendData(void* data, int size)
{
    return sendto(_socket, data, size, 0, (struct sockaddr *)&_destAddress, sizeof (struct sockaddr));
}

int MulticastSocket::receiveData(void *data, int size)
{
    struct pollfd pfd;

    pfd.fd = _socket;
    pfd.events = POLLIN; // check if there is data to read
    int ret = poll(&pfd, 1, 250); // timeout 250 ms

    if (ret == -1)
    {
        perror("poll");
        return -1;
    }
    if (!ret)
    {
        // timeout
        return 0;
    }
    if (pfd.revents & POLL_IN)
    {
        // data available to read - non-blocking recv
        return recv(_socket, data, size, 0);
    }
    return 0;
}

void MulticastSocket::autoSelectInterface()
{
    std::set<std::string> interfaceNames;
    std::string selectedInterfaceName;

    // Create list of valid interface names
    struct ifaddrs* ifa = NULL;
    getifaddrs(&ifa);
    while (ifa)
    {
        if ((ifa->ifa_addr->sa_family == AF_INET) && (!interfaceBlackList.count(ifa->ifa_name)))
        {
            interfaceNames.insert(ifa->ifa_name);
        }
        ifa = ifa->ifa_next;
    }

    // Loop over the priority list and return the first one that is found
    for (auto interfaceName: interfacePriorityList)
    {
        if (interfaceNames.count(interfaceName) != 0)
        {
            selectedInterfaceName = interfaceName;
        }
    }
    
    // If none of the interfaces in the prioritylist is available, choose the first available
    if (selectedInterfaceName.empty())
    {
        if (interfaceNames.size() > 0)
        {
            selectedInterfaceName = *(interfaceNames.begin());
        }
        else
        {
            throw(std::runtime_error("No interfaces available"));
        }   
    }

    _interfaceName = selectedInterfaceName;
}

