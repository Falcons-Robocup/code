""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 
# FALCONS // Jan Feitsma, August 2017



# python includes
# none

# package includes
from utils import Pose



class refboxState():
    def __init__(self):
        self.lastSignal = "undefined" # human readable string such as 'CYAN Goalkick', 'START' etc.
        self.timeStamp = 0.0
        self.phase = "undefined" # 'Pre-game', '1st Half' etc.
        self.active = False # active game time? TODO consider also using preparation time to extract good data?

class robotState():
    def __init__(self, poseTuple, velTuple):
        self.pose = Pose(*poseTuple)
        self.velocity = Pose(*velTuple) # reuse Pose for handy operations
        self.ballEngaged = False

class worldState(): # teamState currently equals worldState, but it could become more (intentions etc.)
    def __init__(self):
        self.valid = False
        self.ball = None # or position (x, y, z), ignore speed for now
        self.obstacles = [] # unused for now
        self.robots = {} # index: physical number 1..X (so idx 0 is dummy); value robotState object
        self.timeStamp = 0.0

class gameState():
    """
    Capture the world state according to a team in an object
    which the analyzer can use effectively (rather than raw json).
    """

    def __init__(self):
        self.teams = []
        self.teams.append(worldState())
        self.teams.append(worldState())
        self.refbox = refboxState()
        self.timeStamp = 0.0
        

