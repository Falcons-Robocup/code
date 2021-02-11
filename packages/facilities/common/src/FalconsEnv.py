# Copyright 2015-2020 Tim Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3


import os, sys
import socket
from FalconsCommon import *

def env_debug():
    """Debugging: dump environment to stdout."""
    d = os.environ
    for k in sorted(d.keys()):
        print("%s=%s" % (k, d[k]))
    
def get_robot_num():
    """Returns robot number, which is an int between and including 1 and MAX_ROBOTS."""
    result = int(os.environ.get('TURTLE5K_ROBOTNUMBER', 0))
    if not result in range(MAX_ROBOTS+1):
        env_debug()
        print("something is wrong with environment, check above")
        print("TURTLE5K_ROBOTNUMBER should be a number between 0 and %s" % (str(MAX_ROBOTS)))
        # TODO link to wiki documentation
        sys.exit(1)
    return result
    
def get_team_name():
    """Returns teamname (either teamA or teamB)."""
    result = os.environ.get('TURTLE5K_TEAMNAME')
    if not result in ["teamA", "teamB"]:
        env_debug()
        print("something is wrong with environment, check above")
        print("TURTLE5K_TEAMNAME should be either teamA or teamB")
        # TODO link to wiki documentation
        sys.exit(1)
    return result

def get_simulated():
    """Returns if we are in simulation mode (see setupEnv.sh)."""
    if get_on_real_robot():
        return False
    if (os.environ.get('SIMULATED') == "1"):
        return True
    return False


def get_ip_address():
    """Returns local host address. 
    This function is intended to be used only by API and 
    modules trying to bind with API (peripherals.py)"""
    return "127.0.0.1"
    
'''#the below function is commented out in china for API had bottleneck with handling data due to the use of IP address and not the local host"
def get_ip_address():
    """Returns IP address of this robot."""
    if get_simulated():
        ip_address = "127.0.0.1"
    else:
        robotnr = get_robot_num()
        ip_address = "%s%s" % (os.getenv("ROBOTNET", "192.168.5.5"), str(robotnr)) 
        #after portugal, use 127.0.0.1 instead of 192.168.5 
    return ip_address
'''
def get_current_ip():
    PROTO = netifaces.AF_INET   # We want only IPv4, for now at least
    # Get list of network interfaces
    # Note: Can't filter for 'lo' here because Windows lacks it.
    ifaces = netifaces.interfaces()
    # Get all addresses (of all kinds) for each interface
    if_addrs = [netifaces.ifaddresses(iface) for iface in ifaces]
    # Filter for the desired address type
    if_inet_addrs = [addr[PROTO] for addr in if_addrs if PROTO in addr]
    iface_addrs = [s['addr'] for a in if_inet_addrs for s in a if 'addr' in s]
    return iface_addrs[-1]
    
def get_pyro_port():
    """Pyro requires a dedicated port, so for simulation each simulated robot must get its own."""
    robotnr = get_robot_num()
    teamname = get_team_name()
    result = 8080
    if teamname == "teamB":
        result += 10
    result += robotnr
    return result
    
def info():
    """Print a oneliner on stdout for debugging."""
    print("%s:  sim=%d  robotnum=%d  teamname=%s  ip=%s  port=%d" % (sys.argv[0], get_simulated(), get_robot_num(), get_team_name(), get_ip_address(), get_pyro_port()))
    
if __name__ == '__main__':
    env_debug()
    

