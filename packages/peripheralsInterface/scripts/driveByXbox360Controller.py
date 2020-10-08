""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3
# Author: Jan Feitsma
# Date: 2018-09-23
#
# This script requires a connected (USB) xbox 360 controller.
# It will feed the commands to motion, it can be even run on coach
# because it uses RTDB.
#
# Keys:
#   left-stick:    drive (forward, backward, strafe)
#   right-stick:   rotate
#   right-trigger: shoot
#   left-trigger:  modify kicker height (hold while shooting)
#   B:             ballhandler on/off
#
# loosely based on: https://pypi.org/project/xbox360controller/  -- no, doesn't work: requires python3 which is incompatible with rtdb2


from __future__ import print_function
import argparse
import signal
import pygame
import sys, time
import pause, datetime
import falconspy
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH


class Button:
    """
    A Button is a controller element which is either pressed or not.
    Examples: A, left-stick (LS), right-bumber (RB), select.
    """
    def __init__(self):
        self.when_pressed = lambda:None
        self.when_released = lambda:None
        self.is_pressed = False

class Axis1:
    """
    An Axis1 is a controller element which has a 1-dimensional value.
    Examples: left-trigger (LT), right-trigger (RT).
    """
    def __init__(self):
        self.when_moved = lambda:None
        self.x = 0.0
        
class Axis2:
    """
    An Axis2 is a controller element which has a 2-dimensional value.
    Examples: left-stick (LS), right-stick (RS).
    """
    def __init__(self):
        self.when_moved = lambda:None
        self.x = 0.0
        self.y = 0.0
        
class Xbox360Controller:
    """
    Maintain live the state of the controller.
    Can fire callbacks upon change.
    Tries to mimick the behavior by https://pypi.org/project/xbox360controller.
    Uses pygame to connect to the controller and handle events.
    """
    def __init__(self, index=0, axis_threshold=0.2):
        self.axis_threshold = axis_threshold
        self.live_dump_mode = False
        self.callback = lambda s: None
        # setup controller elements
        self.button_a = Button()
        self.button_b = Button()
        self.button_x = Button()
        self.button_y = Button()
        self.button_lb = Button() # left bumper
        self.button_rb = Button() # right bumper
        self.button_ls = Button() # left stick
        self.button_rs = Button() # right stick
        self.button_select = Button()
        self.button_start = Button()
        self.button_mode = Button() # a.k.a. globe
        self.buttons = {"A": self.button_a, "B": self.button_b, "X": self.button_x, "Y": self.button_y,
                        "LB": self.button_lb, "RB": self.button_rb, "LS": self.button_ls, "RS": self.button_rs,
                        "select": self.button_select, "start": self.button_start, "mode": self.button_mode}
        self.axis_lt = Axis1()
        self.axis_rt = Axis1()
        self.axis_lt.x = -1.0
        self.axis_rt.x = -1.0
        self.axis_ls = Axis2()
        self.axis_rs = Axis2()
        # store initial axis values, needed for delta tracking using threshold
        self.prev = [0.0] * 6
        # initialize controller
        pygame.init()
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        if len(joysticks) < 1:
            raise Exception("no joysticks")
        for idx in range(len(joysticks)):
            print("detected joystick {}: {}".format(idx, joysticks[index].get_name()))
        print("using joystick {}".format(index))
        self.joystick = joysticks[index]
        self.joystick.init()
        self.frequency = 30.0 # Hz
        
    def echo_onchange(self):
        """
        Install callbacks to echo state change. Useful for testing controller state and developing.
        """
        for b in self.buttons.keys():
            # mind b=b to bind evaluation, otherwise python will behave lazy...
            self.buttons[b].when_pressed = lambda b=b: print("button {} pressed".format(b))
            self.buttons[b].when_released = lambda b=b: print("button {} released".format(b))
        self.axis_lt.when_moved = lambda: print("trigger LT got value {:6.3f}".format(self.axis_lt.x))
        self.axis_rt.when_moved = lambda: print("trigger RT got value {:6.3f}".format(self.axis_rt.x))
        self.axis_ls.when_moved = lambda: print("stick LS got value ({:6.3f},{:6.3f})".format(self.axis_ls.x, self.axis_ls.y))
        self.axis_rs.when_moved = lambda: print("stick RS got value ({:6.3f},{:6.3f})".format(self.axis_rs.x, self.axis_rs.y))
        
    def live_dump(self):
        self.live_dump_mode = True
        
    def __str__(self):
        s = "A={:d} B={:d} X={:d} Y={:d}".format(self.button_a.is_pressed, self.button_b.is_pressed, self.button_x.is_pressed, self.button_y.is_pressed)
        s += " LB={:d} RB={:d} LS={:d} RS={:d}".format(self.button_lb.is_pressed, self.button_rb.is_pressed, self.button_ls.is_pressed, self.button_rs.is_pressed)
        s += " sel={:d} st={:d} m={:d}".format(self.button_select.is_pressed, self.button_start.is_pressed, self.button_mode.is_pressed)
        s += " LT={:6.3f} RT={:6.3f}".format(self.axis_lt.x, self.axis_rt.x)
        s += " LS=({:6.3f},{:6.3f}) RS=({:6.3f},{:6.3f})".format(self.axis_ls.x, self.axis_ls.y, self.axis_rs.x, self.axis_rs.y)
        return s
        
    def run(self):
        # loop using pygame
        dt = datetime.timedelta(seconds=(1.0 / self.frequency))
        t = datetime.datetime.now()
        try:
            # setup pygame index maps
            buttons = {0: self.button_a, 1: self.button_b, 2: self.button_x, 3: self.button_y,
                        4: self.button_lb, 5: self.button_rb, 6: self.button_select, 7: self.button_start,
                        8: self.button_mode, 9: self.button_ls, 10: self.button_rs}
            # iterate
            done = False
            while not done:
                # initialize the list of callbacks to fire
                callbacks = set()
                # process all events
                for event in pygame.event.get():
                    # done?
                    if event.type == pygame.QUIT:
                        done = True
                    # possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
                    if event.type == pygame.JOYAXISMOTION:
                        ax = None
                        if event.axis == 0:
                            self.axis_ls.x = event.value
                            ax = self.axis_ls
                        elif event.axis == 1:
                            self.axis_ls.y = event.value
                            ax = self.axis_ls
                        elif event.axis == 2:
                            self.axis_lt.x = event.value
                            ax = self.axis_lt
                        elif event.axis == 3:
                            self.axis_rs.x = event.value
                            ax = self.axis_rs
                        elif event.axis == 4:
                            self.axis_rs.y = event.value
                            ax = self.axis_rs
                        elif event.axis == 5:
                            self.axis_rt.x = event.value
                            ax = self.axis_rt
                        # check against delta threshold and if needed update previous value
                        delta = abs(self.prev[event.axis] - event.value)
                        if delta > self.axis_threshold:
                            callbacks.add(ax.when_moved)
                            self.prev[event.axis] = event.value
                    elif event.type == pygame.JOYBUTTONDOWN:
                        button = buttons[event.button]
                        if not button.is_pressed:
                            callbacks.add(button.when_pressed)
                        button.is_pressed = True
                    elif event.type == pygame.JOYBUTTONUP:
                        button = buttons[event.button]
                        if button.is_pressed:
                            callbacks.add(button.when_released)
                        button.is_pressed = False
                    elif event.type == pygame.JOYHATMOTION:
                        pass # D-pad not supported
                    else:
                        raise Exception("cannot process joystick event: " + str(event))
                # fire all callbacks
                for f in callbacks:
                    f()
                # dump?
                if self.live_dump_mode:
                    print(self)
                # callback
                self.callback(self)
                # sleep until
                t += dt
                pause.until(t)
        except KeyboardInterrupt:
            pass

class xRelay:
    def __init__(self, robotId, joystickIndex=0):
        # setup RTDB
        self.robotId = robotId # TODO: allow live toggle via select button?
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False)
        self.rtdb2Store.refresh_rtdb_instances()
        # ballhandler enable/disable events
        self.enable_bh = False
        self.toggle_bh()
        # setup controller callbacks
        self.controller = Xbox360Controller(joystickIndex)
        self.controller.button_b.when_pressed = self.toggle_bh
        self.controller.callback = self.process_state
        # motion limiters and timing
        self.xy_max_speed = 1.2
        self.xy_acceleration = 2.0
        self.xy_deadzone = 0.3
        self.vx = 0.0
        self.vy = 0.0
        self.rz_max_speed = 2.3
        self.rz_acceleration = 3.0
        self.rz_deadzone = 0.5
        self.vrz = 0.0
        self.dt = 1.0 / self.controller.frequency
        # shooting
        self.rt = -1.0
        self.allow_lobshots = False
        self.shoot_power_min = 20
        self.shoot_power_max = 60
        self.shoot_power_scale = 60
        self.shoot_height_scale = 90
        self.shoot_height_max = 180
        # advanced actions
        self.action = ""

    def clip(self, v, lim):
        return min(max(v, -lim), lim)
        
    def toggle_bh(self):
        self.enable_bh = not self.enable_bh
        print("ballHandlers " + ["off", "on"][self.enable_bh])

    def process_state(self, controller_state):
        # helper function, very basic motion controller
        def calc(current_setpoint, axis_input, speed_limit, acc_limit, deadzone):
            """
            Calculate new setpoint based on axis input, current setpoint and motion limiters.
            """
            target = axis_input * speed_limit
            if abs(target) < deadzone:
                return 0.0 # TODO: is abrupt braking OK?
            if target > current_setpoint:
                return min(current_setpoint + self.dt * acc_limit, target)
            return max(current_setpoint - self.dt * acc_limit, target)
        vx = calc(self.vx, controller_state.axis_ls.x, self.xy_max_speed, self.xy_acceleration, self.xy_deadzone)
        vy = calc(self.vy, -controller_state.axis_ls.y, self.xy_max_speed, self.xy_acceleration, self.xy_deadzone) # inverted axis!
        vrz = calc(self.vrz, -controller_state.axis_rs.x, self.rz_max_speed, self.rz_acceleration, self.rz_deadzone) # rz is defined counter-clockwise
        v_string = "vx={:6.2f} vy={:6.2f} vrz={:6.2f}".format(vx, vy, vrz)
        v_zero = "vx={:6.2f} vy={:6.2f} vrz={:6.2f}".format(0, 0, 0)
        if v_string != v_zero:
            print("{:.3f} {}".format(time.time(), v_string))
        # store speed setpoints
        self.vx = float(vx)
        self.vy = float(vy)
        self.vrz = float(vrz)
        # kicker setpoints
        self.kicker_height = float(min((controller_state.axis_lt.x + 1.0) * 0.5 * self.shoot_height_scale, self.shoot_height_max))
        self.kicker_power = float(0.0)
        if not self.allow_lobshots:
            self.kicker_height = float(0.0)
        # shoot if RT is being released
        new_rt = controller_state.axis_rt.x
        if (new_rt < self.rt - 0.2):
            # kaboom!
            # TODO: check if we have the ball
            self.kicker_power = float(min(max((self.rt + 1.0) * 0.5 * self.shoot_power_scale, self.shoot_power_min), self.shoot_power_max))
            print("shooting with height {:6.2f} and power {:6.2f}".format(self.kicker_height, self.kicker_power))
            self.rt = -1.0 # reset to avoid immediate reshoot
        else:
            self.rt = new_rt
        # check for special actions
        self.action = ""
        if controller_state.button_a.is_pressed:
            self.action = "getBall"
        if controller_state.button_x.is_pressed:
            self.action = "passToTeamMember"
        if controller_state.button_y.is_pressed:
            self.action = "shootAtGoal"
        # serialize and put into RTDB
        item = [self.robotId, [self.vx, self.vy, self.vrz], self.enable_bh, self.kicker_height, self.kicker_power, self.action]
        self.rtdb2Store.put(0, "JOYSTICK_CONTROL_" + str(self.robotId), item)
        # shooting?
        if self.kicker_power > 0.0:
            time.sleep(0.5) # wait for ball to leave
        
    def run(self):
        self.controller.run()
            
def main(robotId, joystickIndex=0):
    # RTDB relay
    if 1:
        xRelay(robotId, joystickIndex).run()
    else:
        # dev mode: echo controller state change
        controller = Xbox360Controller()
        #controller.echo_onchange()
        controller.live_dump()
        controller.run()
        
if __name__ == "__main__":
    # Argument parsing.
    descriptionTxt = 'Control given robot using a XBOX controller.\n(Or any other regular controller, tooling to be generalized.)\n'
    exampleTxt = 'Example: driveByXbox360Controller.py 2\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-i', '--index', help='joystick index to use', type=int, default=0)
    parser.add_argument('robotId', help='target robot', type=int)
    args       = parser.parse_args()

    # execute only if run as a script
    main(args.robotId, args.index)
    
