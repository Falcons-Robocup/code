""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python layer for simulator control.
# Shared by simControl shell wrapper and Test2FalconsControl
#
# Jan Feitsma, 2016-03-11



import sys, os
import time
sys.path.append("/home/robocup/falcons/code/packages/coachCommands")
from udpInterface import udpInterface



# trick based on http://thesmithfam.org/blog/2012/10/25/temporarily-suppress-console-output-in-python/#
from contextlib import contextmanager
@contextmanager
def suppress_output():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr


def createGlobalConfigOverride():
    cmd = "cp " + os.getenv("TURTLEROOT") + "/config/simConfigOverride.txt ~/configOverride.txt"
    os.system(cmd)

def simInit(teams = ["A", "B"], robotnums = [1, 2, 3, 4, 5, 6]):
    """
    Initialize a simulation.
    TeamA is "our" team, teamB is a second instance but with less detail (no visualizer etc).
    Start team-level processes to bootstrap the teams.
    Also start robotControl daemons, which subsequently should be activated.
    """
    createGlobalConfigOverride()
    os.putenv("SIMULATED", "1")
    with suppress_output():
        cmd = "simInit"
        if "B" in teams:
            cmd += " -b"
        for robotnum in robotnums:
            cmd += " " + str(robotnum)
        os.system(cmd)
            
            
def simStop():
    """
    Shutdown a simulation.
    This is done bluntly by killing all processes.
    """
    with suppress_output():
        os.system("simKill")


def simActivate(robotkey):
    """
    Activate robot by starting all of its processes and registering it in simulator.
    """
    with suppress_output():
        team = robotkey[0]
        robotnum = int(robotkey[1])
        os.putenv("CONTEXTKEY", robotkey)
        os.putenv("SIMULATED", "1")
        os.system("robotControl start")
                     

def simTeleportBall(x, y, vx = 0, vy = 0):
    """
    Teleport ball, optionally with speed. Coordinates in ACS.
    """
    x = float(x)
    y = float(y)
    vx = float(vx)
    vy = float(vy)
    os.system('rosservice call /simulator/teleportball "{x: %.3f, y: %.3f, vx: %.3f, vy: %.3f}"' % (x, y, vx, vy))    


def simMove(teamname, robotnum, x, y, phi):
    """
    Send a MOVE command.
    """
    assert(teamname == "teamA") # teamB commandGUI not yet supported
    # sanitize inputs
    robotnum = int(robotnum)
    x = float(x)
    y = float(y)
    phi = float(phi)
    command = "target %.3f %.3f %.3f" % (x, y, phi)
    os.environ["SIMULATED"] = "1"
    udpInterface().sendTo(robotnum, command)


def simCommand(commandStr):
    """
    Parse a command string, call appropriate function.
    """
    words = commandStr.split()
    if len(words) == 0:
        print "missing argument"
        sys.exit(1)
    mode = words[0]
    args = words[1:]
    if mode == "init":
        simInit()
    elif mode == "stop":
        simStop()
    elif mode == "activate":
        simActivate(args[0])
    elif mode == "teleportball":
        simTeleportBall(*args)
    elif mode == "move":
        simMove(*args)
    else:
        print "unknown mode: " + mode
        sys.exit(1)
        
        
if __name__ == '__main__':
    simCommand(" ".join(sys.argv[1:]))
    sys.exit(0)
        
