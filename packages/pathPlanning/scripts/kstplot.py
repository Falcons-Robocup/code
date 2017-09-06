""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import time
import subprocess
import shlex
import sys
import argparse

sys.path.append( "/home/robocup/falcons/code/packages/facilities/common/src" )
import FalconsEnv

# All topics to log to file.
topics = []
topics.append( "g_pp_plotdata"          ) # Plotdata by pathplanning
topics.append( "g_target"               ) # Target published by teamplay, read by pathplanning
topics.append( "g_robotspeed"           ) # Velocity published by pathplanning, read by peripheralsInterface
topics.append( "t_motor_pid_params"     ) # Data per motor, published by peripheralsInterface
topics.append( "t_motor_robot_speed"    ) # Velocity as output from motors, published by peripheralsInterface

# All processes that log to file will be placed in this list
processes = []

try:

    # Parse arguments
    parser = argparse.ArgumentParser(description='Logs the output from several topics to a file. KST plot can plot realtime from file.')
    parser.add_argument('-r', '--robot', help='The robot number of which topics will be written to file. Optional.')
    args = parser.parse_args()

    # Parse the robot number.
    if args.robot is None:
        robotnum = FalconsEnv.get_robot_num()

        if robotnum == 0:
            print "Unable to get robot number. Is simulation running and did you forget to give the robot number as argument?"
            parser.print_usage()
            exit()
    else:
        robotnum = args.robot
    print "Robot identified as number: '%s'" % robotnum

    # Log all topics to file
    for topic in topics:
        cmd = "rostopic echo -p /teamA/robot%s/%s > /home/robocup/kst_templates/%s.txt" % (robotnum, topic, topic)
        print cmd
        p = subprocess.Popen(cmd, shell=True)
        processes.append( p )

    print "Logging to files in /home/robocup/kst_templates/... Close me with Ctrl+C."

    while True:
        time.sleep(1)

except KeyboardInterrupt:

    print "Closing..."

    for p in processes:
        p.terminate()

    sys.exit()

