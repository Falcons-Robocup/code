""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from monitorLocalization import monitorLocalization



def kpiLocalization():
    """
    Measure localization accuracy and repro while robot is standing still.
    Requires manual positioning on the field!
    """
    def prompt(msg):
        log(msg, 0)
        log("[hit enter to continue]", 0)
        raw_input()
    def doit(expected):
        move(*expected)
        prompt("fine-place the robot at (%6.2f,%6.2f,%6.2f)" % expected)
        sleep(1)
        log("measuring ...", 0)
        timeout = 10
        startThread(monitorLocalization, timeout, expected)
        sleep(timeout + 2) # sleep a bit longer
    # first move to center
    doit((0, 0, 0.5*pi))
    # right-hand side line
    doit((6, 0, 0.5*pi))
    # back right-hand side corner
    doit((6, -9, 0.5*pi))
    # goalie position
    doit((0, -8, 0.5*pi))
    # in the middle of nowhere
    doit((3, -4, 0.5*pi))
    
    

