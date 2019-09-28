""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import argparse
from TrajectoryPlanner_2ndorder import TrajectoryPlanner

class SimpleRobot():

    def __init__(self):
        pass

    def compute(self, x, a, t):
        # x = [v, s]
        # a = [a]

        # result = [a, v]
        return np.append(a, x[0:1])

class RobotModel():

    def __init__(self):
        self.trajectory_planner = TrajectoryPlanner()
        self.robot_model = SimpleRobot()

    def computeTrajectory(self, s_0, v_0, s_end, v_end, v_max, a_max):

        self.periods = self.trajectory_planner.computeFullTrajectory(s_0, v_0, s_end, v_end, v_max, a_max)
        print self.periods


    def compute(self, x, t):

        # a = [a]
        a = self.trajectory_planner.plot(t, self.periods)

        # x = [v, s]
        x = self.robot_model.compute(x, a, t)
        # x = [a, v]

        # When returning x, all will be integrated to have the following result:
        # result = [v, s]

        return x

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("s_0", help="Initial position")
    parser.add_argument("v_0", help="Initial velocity")
    parser.add_argument("s_end", help="Target position")
    parser.add_argument("v_end", help="Target velocity")
    parser.add_argument("v_max", help="Maximum velocity")
    parser.add_argument("a_max", help="Maximum acceleration")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    s_0 = float(args.s_0)
    v_0 = float(args.v_0)
    s_end = float(args.s_end)
    v_end = float(args.v_end)
    v_max = float(args.v_max)
    a_max = float(args.a_max)

    if args.plot:
        import scipy.integrate
        import matplotlib.pyplot as plt
        import numpy as np

        model = RobotModel()
        model.computeTrajectory( s_0, v_0, s_end, v_end, v_max, a_max )

        plot_duration = sum(model.periods)
        stepsize = 0.001
        t = np.arange(0.0, plot_duration, stepsize)

        (state, info) = scipy.integrate.odeint(model.compute, [v_0, s_0], t, full_output=True, hmax=0.01)
        v = (state[:, 0])
        s = (state[:, 1])

        a = []
        for step in t:
            a.append( model.trajectory_planner.plot(step, model.periods)[0] )

        plt.plot(t[0:len(s)], s, label="pos")
        plt.plot(t[0:len(v)], v, label="vel")
        plt.plot(t[0:len(a)], a, label="acc")
        plt.legend()
        plt.grid()
        plt.show()

    else:

        t = TrajectoryPlanner()
        result = t.computeFullTrajectory(s_0, v_0, s_end, v_end, v_max, a_max)

        if result == -1:
            print "Not supported -- skipping."
        else:
            print t.plot(0.0, result)[0]
