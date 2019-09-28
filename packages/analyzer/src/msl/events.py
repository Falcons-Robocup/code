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
import time
import math

# package includes
# none

# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


# only messages with level <= SHOW_LEVEL are printed
SHOW_LEVEL = 9
T0 = 0
INDENT = True

# internal buffer to suppress repeating messages
# key: message string (exact), value: (timestamp, timeout)
_buffer = {}


def info(message, level=0, timeStamp=None, timeout=0):
    _handle(message, level, timeStamp, timeout, "INFO")

def warning(message, level=0, timeStamp=None, timeout=0):
    _handle(message, level, timeStamp, timeout, "WARN")

# internals below
    
def _handle(message, level, timeStamp, timeout, type):
    global _buffer
    if timeStamp == None:
        timeStamp = time.time()
        timeStampPrint = None
    else:
        t = (timeStamp - T0)
        if t > 1e7:
            # assume absolute seconds
            timeStampPrint = '%.1f' % (t)
        else:
            # pretty-print MM:SS.x; analyzer is expected to set T0 to say first refbox event
            m = math.floor(t / 60)
            t -= 60 * m
            s = math.floor(t)
            f = math.floor(10 * (t - s))
            timeStampPrint = '%3d:%02d.%d' % (m, s, f)
    show = True
    if _buffer.has_key(message):
        if timeStamp - _buffer[message][0] < timeout:
            show = False
    if level > SHOW_LEVEL:
        show = False
    if show:
        indent = ''
        if INDENT:
            indent = ''.join([' '] * level)
        if timeStampPrint != None:
            timeStampPrint = "%s: " % (timeStampPrint)
        else:
            timeStampPrint = ""
        _buffer[message] = (timeStamp, timeout)
        messagePrint = "%s: %s%s%s" % (type, timeStampPrint, indent, message)
        trace("%s", messagePrint)
        print messagePrint
    _cleanup(timeStamp)

    
def _cleanup(timeStamp):
    global _buffer
    for k in _buffer.keys():
        if timeStamp - _buffer[k][0] >= _buffer[k][1]:
            del _buffer[k]

