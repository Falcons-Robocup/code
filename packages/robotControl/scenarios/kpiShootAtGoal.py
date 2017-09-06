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


def kpiShootAtGoal():
    while True:
        # Place kicker between center and opponent goal
        action("positionBeforePOI source=P_OPP_GOALLINE_CENTER destination=P_CENTER distance=5 facing=P_OPP_GOALLINE_CENTER")

        while not teamSeesBall():
            # Stay here until we see a ball with 1m of the center point
            sleep(1)

        # Found ball near center point. Prepare for kickoff.
        behavior("getBall role=attackerMain")

        sleep(5)

        # Perform kickoff
        behavior("shootAtGoal role=attackerMain")

        sleep(10)

# This scenario tests shooting at the goal, turning from different angles.
# If the full scenario runs, it will shoot at goal after turning [180, 150, 120, 90, 60, 30, 0] degrees
def kpiShootAtGoal_LongStroke_LongDistance():

    # starting angle
    angle = -1.67
    step_size = pi / 6.0

    while True:

        print "Starting scenario..."

        #Position robot on center
        move(0, 0, angle)

        print "Moved to position."
        
        # Wait until robot has ball
        while not hasBall():
            move(0, 0, angle)
            print "Waiting for ball at angle %12.9f..." % angle
            sleep(1)

        print "Got ball. Aiming towards goal."

        # Robot has the ball. Turn towards goal and shoot.
        actionBlocking("move target=robot facing=P_OPP_GOALLINE_CENTER")

        print "Successfully aiming at goal. Shooting..."

        actionBlocking("shoot shootType=shootTowardsGoal")

        print "Shoot done."

        sleep(5)

        angle = angle + step_size
