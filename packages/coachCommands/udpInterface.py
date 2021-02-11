# Copyright 2015-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Broadcast commands to robots.
#
# 2015-12-15 JFEI creation


import sys, os
import socket
import time
import struct
sys.path.append("/home/robocup/falcons/code/packages/facilities/common/src")
from FalconsTrace import trace



def burstSend(sock, ip, port, message, count):
    for c in range(count):
        sock.sendto(message, (ip, port))
        time.sleep(0.001)
        
def getPort(robotnum):
    # TODO for C++ this is centralized in ports.hpp... 
    # this is duplicate and sensitive to change :( .. DANGER
    port = 10000 + 1000 * robotnum + 1
    if os.environ.has_key("SIMULATED") and os.environ["SIMULATED"] == "1":    
        port += 500
    return port
    
class udpInterface():

    def __init__(self):
        self.targetRobots = []
        self.hostname = socket.gethostname()
        self.address = "224.16.32.74" # TODO for C++ this is centralized in addresses.hpp... this is duplicate :(
        self.burstcount = 5
        # Setup the socket
        ttl = 2
        if os.environ.has_key("SIMULATED") and os.environ["SIMULATED"] == "1":
            ttl = 0
        addrinfo = socket.getaddrinfo(self.address, None)[0]
        s = socket.socket(addrinfo[0], socket.SOCK_DGRAM)
        if True:
            # multicast
            ttl_bin = struct.pack('@i', ttl)
            if addrinfo[0] == socket.AF_INET: # IPv4
                s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)
            else:
                s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, ttl_bin)
        self.socket = s

    def send(self, command, targetRobots = None):
        """
        Send a command to all selected robots.
        """
        # if not specified, use internal list
        if targetRobots == None:
            targetRobots = self.targetRobots
        for r in targetRobots:
            self.sendTo(r, command)
            
    def sendTo(self, robotnum, command):
        """
        Send a command to a dedicated robot.
        """
        # prepend hostname to achieve claiming - robotControl listener will disregard
        # incoming messages from other hosts, if any
        message = "from " + self.hostname + " " + command
        trace("sendTo addr=%s port=%d message='%s'" % (self.address, getPort(robotnum), message))
        burstSend(self.socket, self.address, getPort(robotnum), message, self.burstcount)



