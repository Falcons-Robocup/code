""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotRosInterface import *



def monitorSpeed(y1, y2):
    """
    Monitor function: report the time spent between lines y=y1 and y=y2.
    Typically running in parallel while executing some function.
    Requires robot to be below the y1 line.
    """
    t1 = None
    t2 = None
    dt = 0.05
    pos1 = None
    pos2 = None
    pos = getPosition()
    assert(pos[1] < y1)
    while True:
        sleep(dt)
        pos = getPosition()
        if (pos[1] >= y1) and t1 == None:
            t1 = time()
            pos1 = pos
            print "y1 crossed" # TODO remove
        if (pos[1] >= y2):
            t2 = time()
            pos2 = pos
            break
    # calculate speed
    distance = sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    speed = distance / (t2 - t1)
    print "measured speed: %6.2f m/s" % (speed)
