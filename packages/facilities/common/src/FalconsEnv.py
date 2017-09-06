""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python


import os, sys
import socket

def env_debug():
    """Debugging: dump environment to stdout."""
    d = os.environ
    for k in sorted(d.keys()):
        print "%s=%s" % (k, d[k])
       
def get_on_real_robot():
    """Return true if called on real robot (by looking at hostname)."""
    return "FALCON" in socket.gethostname()
    
def get_robot_num():
    """Returns robot number, which is an int between and including 1 and 6."""
    result = int(os.environ.get('TURTLE5K_ROBOTNUMBER', 0))
    if not result in [0, 1, 2, 3, 4, 5, 6]:
        env_debug()
        print "something is wrong with environment, check above"
        print "TURTLE5K_ROBOTNUMBER should be a number between 0 and 6"
        # TODO link to wiki documentation
        sys.exit(1)
    return result
    
def get_team_name():
    """Returns teamname (either teamA or teamB)."""
    result = os.environ.get('TURTLE5K_TEAMNAME')
    if not result in ["teamA", "teamB"]:
        env_debug()
        print "something is wrong with environment, check above"
        print "TURTLE5K_TEAMNAME should be either teamA or teamB"
        # TODO link to wiki documentation
        sys.exit(1)
    return result

def get_simulated():
    """Returns if we are in simulation mode (see falconsconfig.sh)."""
    return os.environ.get('SIMULATED') == "1"


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
    print "%s:  sim=%d  robotnum=%d  teamname=%s  ip=%s  port=%d" % (sys.argv[0], get_simulated(), get_robot_num(), get_team_name(), get_ip_address(), get_pyro_port())
    
if __name__ == '__main__':
    env_debug()
    

