""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
import threading

# Simulate real-world kickoff as accurate as possible.
# Pick a random starting position, and make sure the robots need to turn around to find their kickoff position.
def kpiKickoffKicker():
    while True:
        # Place kicker between center and opponent goal
        action("positionBeforePOI source=P_OPP_GOALLINE_CENTER destination=P_CENTER distance=2 facing=P_OPP_GOALLINE_CENTER")

        while not teamSeesBall() or not ballCloseBy(x=0.0, y=0.0, xyTol=1.0):
            # Stay here until we see a ball with 1m of the center point
            sleep(1)

        # Found ball near center point. Prepare for kickoff.
        behavior("ownKickoffPrepare role=defenderMain")

        sleep(5)

        # Perform kickoff
        behavior("ownKickoffExecute role=defenderMain")

        sleep(10)
    

def kpiKickoffReceiver():
    while True:
        # Place receiver between center and own goal
        action("positionBeforePOI source=P_OWN_GOALLINE_CENTER destination=P_CENTER distance=2 facing=P_OWN_GOALLINE_CENTER")

        while not teamSeesBall() or not ballCloseBy(x=0.0, y=0.0, xyTol=1.0):
            # Stay here until we see a ball with 1m of the center point
            sleep(1)

        # Found ball near center point. Prepare for kickoff.
        behavior("ownKickoffPrepare role=attackerMain")

        sleep(5)

        # Perform kickoff
        behavior("ownKickoffExecute role=attackerMain")

        sleep(10)
