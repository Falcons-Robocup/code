""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import random
# load basic commands
from robotActions import *

def penalties():
    # settings
    searchWayPoints = ((0, 3, 1.57), (3, 6, 1.57), (3, 0, 1.57), (-3, 0, 1.57), (-3, 6, 1.57))
    homePos = (0, 4, 1.57)
    penaltyPos = (0, 6, 1.57)
    angles = (1.57, 1.80, 1.34)
    wiggleSleep = 0.5
    shootPower = 80
    # helper functions
    def searchBall():
        idx = 0
        while not teamSeesBall():
            # move to wayPoint
            move(*searchWayPoints[idx])
            # next 
            idx = (1 + idx) % len(searchWayPoints)
    def getBall():
        # do not attack goalkeeper, wait for the ball to be clear
        while ballCloseBy(0, 9, 2):
            sleep(1)
        getBall()
    def moveHome():
        move(*homePos)
    def fetchBall():
        searchBall()
        getBall()
        moveHome()
    def wiggle(n):
        for i in range(n):
            pos = list(penaltyPos)
            pos[2] = angles[i % len(angles)]
            move(*pos)
            sleep(wiggleSleep)
    def takePenalty():
        move(*penaltyPos)
        # wiggle around a little, to provoke a response from the keeper
        wiggle(random.randint(1, 7))
        safeKick(shootPower)
        sleep(3)
    # iterate
    while True:
        fetchBall()
        takePenalty()
        

