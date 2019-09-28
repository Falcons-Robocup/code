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



# default sample rate [Hz]
DEFAULT_SAMPLE_RATE         = 5.0

# threshold for data quality - rules state that teams should provide data at 10Hz; we sample at 5Hz
DATA_QUALITY_TIME_THRESHOLD = 0.3

# threshold for teleportation check: if a robot suddenly moves more than X meters, we register a teleportation
TELEPORT_DISTANCE           = 2.0
# TODO phi check?

# ball localization threshold for center area
BALL_CENTER_BAND            = 1.0

# pass detection, shots on goal and other ballPossession-related analysis
SELF_PASS_TIMEOUT           = 1.0 # maximum number of seconds between losing and regaining ball possession
PASS_AIM_THRESHOLD          = 0.4 # cone angle (radians) within which pass receiver must be located
REGULAR_PASS_TIMEOUT        = 3.0 # maximum number of seconds between losing and regaining ball possession
SHOOT_TIMEOUT               = 8.0 # time after losing ball and concluding it must have been a goal attempt
BALL_STEAL_TIMEOUT          = 3.0 # if slower, then it doesn't count as
GOAL_DX_THRESHOLD           = 1.0 # aiming at goal posts with a tolerance +/- dx

