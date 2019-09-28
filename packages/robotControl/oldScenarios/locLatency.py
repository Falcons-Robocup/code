""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotInterface import *
from robotActions import *



# settings
POSITION_1 = (-3, 1, 0)
POSITION_2 = ( 3, 1, 0)
ITERATIONS = 3

def locLatency():
    """
    Measure localization latency (of vision, w.r.t. encoders).
    """
    
    for it in range(ITERATIONS):
        restoreConfig("worldModelNode/localization")
        sleep(1)
        move(*POSITION_1)
        sleep(1)
        setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0}) # we will be driving on encoders
        sleep(1)
        move(*POSITION_2)
        sleep(1)
        restoreConfig("worldModelNode/localization")
        sleep(1)
        move(*POSITION_2)
        sleep(1)
        setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0}) # we will be driving on encoders
        sleep(1)
        move(*POSITION_1)
        sleep(1)
        
    print ""
    print "to analyze and plot: use either kstplot (configuration stored on pingu) or tracePlot"
    print ""

