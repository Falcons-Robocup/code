""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import enum

class PlotData(enum.Enum):
    PLOT_ROBOT_VEL = 1,
    PLOT_ROBOT_POS = 2

plotDataMapping = {}

plotDataMapping[PlotData.PLOT_ROBOT_VEL] = \
{
    # m1.setpoint, m2.setpoint, m3.setpoint, m1.feedback, m2.feedback, m3.feedback
    "DIAG_PERIPHERALSINTERFACE": ["['speed_vel'][0]", "['speed_vel'][1]", "['speed_vel'][2]", "['feedback_vel'][0]", "['feedback_vel'][1]", "['feedback_vel'][2]"],

    # robot_vx.setpoint, robot_vy.setpoint, robot_vRz.setpoint
    "ROBOT_VELOCITY_SETPOINT": ["[0]", "[1]", "[2]"],

    # robot_vx.feedback, robot_vy.feedback, robot_vRz.feedback
    "ROBOT_VELOCITY_FEEDBACK": ["[0]", "[1]", "[2]"]
}

plotDataMapping[PlotData.PLOT_ROBOT_POS] = \
{
    "ROBOT_STATE": ["[2][0]", "[2][1]", "[2][2]"]
}
