""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from monitorLocalization import monitorLocalization

def gridScan(xLim=6.3, yLim=8, numX=6, numY=8, autoContinue='n'):
    """
    This is a scenario to drive the robot in a grid position over the field
    """

    # sanitize inputs
    xLim = float(xLim)
    yLim = float(yLim)
    numX = int(numX)
    numY = int(numY)

    def doit():
        sleep(1)
        log("measuring ...", 0)
        timeout = 10
        startThread(monitorLocalization, timeout)
        sleep(timeout + 2) # sleep a bit longer

    # calculate step sizes
    stepY = 2.0 * yLim / (1.0 * numY)
    stepX = 2.0 * xLim / (1.0 * numX)

    # loop over y
    direction = 1
    for iY in range(numY+1):
        y = -yLim + stepY * iY
        # loop over x
        for iX in range(numX+1):
            x = direction * (-xLim + stepX * iX)
            # Move to next position and calculate new x setpoint
            move(x, y)
            log("Position is: (%2.1f, %2.1f)" % (x, y))
            # Wait for user input or continue automaticly
            if autoContinue in ['n', 'N', 'no', 'No', 'NO']:
               doit()
            else:
               answer = raw_input("Press enter to continue")
        # toggle direction
        direction *= -1

    print "Grid completed"
        

