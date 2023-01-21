# Copyright 2020-2021 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import enum

class PlotData(enum.Enum):
    PLOT_ROBOT_VEL = 1,
    PLOT_BH = 2,
    PLOT_LATENCY = 3

plotDataMapping = {}

plotDataMapping[PlotData.PLOT_ROBOT_VEL] = \
{
    #### Robot State
    # robot.pos.x, robot.pos.y, robot.pos.Rz, robot.vel.x, robot.vel.y, robot.vel.Rz
    "ROBOT_STATE": ["[2][0]", "[2][1]", "[2][2]", "[3][0]", "[3][1]", "[3][2]"],


    #### Robot PosVel Setpoint
    "ROBOT_POSVEL_SETPOINT": ["[1][0]", "[1][1]", "[1][2]", "[2][0]", "[2][1]", "[2][2]"],


    #### Robot Velocity
    # robot_vx.setpoint, robot_vy.setpoint, robot_vRz.setpoint
    "ROBOT_VELOCITY_SETPOINT": ["[0]", "[1]", "[2]"],

    # robot_vx.feedback, robot_vy.feedback, robot_vRz.feedback
    "ROBOT_VELOCITY_FEEDBACK": ["[0]", "[1]", "[2]"],


    #### Motor Velocity
    # m1.setpoint, m2.setpoint, m3.setpoint
    "MOTOR_VELOCITY_SETPOINT": ["[0]", "[1]", "[2]"],

    # m1.feedback, m2.feedback, m3.feedback
    "MOTOR_FEEDBACK": ["[0][1]", "[1][1]", "[2][1]"],

    # m1.setpoint, m2.setpoint, m3.setpoint, m1.feedback, m2.feedback, m3.feedback, m1.pid_output, m2.pid_output, m3.pid_output, m1.error, m2.error, m3.error, m1.integral, m2.integral, m3.integral, m1.derivative, m2.derivative, m3.derivative, bh_left.angle, bh_right.angle, bh_left.tacho_zero, bh_right.tacho_zero, bh_left.tacho, bh_right.tacho, bh_left.pid_output, bh_right.pid_output, bh_left.error, bh_right.error, bh_left.integral, bh_right.integral, bh_left.pwm, bh_right.pwm
    "DIAG_PERIPHERALSINTERFACE": ["['velocity_setpoint'][0]", "['velocity_setpoint'][1]", "['velocity_setpoint'][2]", "['velocity_feedback'][0]", "['velocity_feedback'][1]", "['velocity_feedback'][2]", "['motor_pid_output'][0]", "['motor_pid_output'][1]", "['motor_pid_output'][2]", "['motor_error'][0]", "['motor_error'][1]", "['motor_error'][2]", "['motor_integral'][0]", "['motor_integral'][1]", "['motor_integral'][2]", "['motor_derivative'][0]", "['motor_derivative'][1]", "['motor_derivative'][2]", "['bh_angle'][0]", "['bh_angle'][1]", "['bh_tacho_zero'][0]", "['bh_tacho_zero'][1]", "['bh_tacho'][0]", "['bh_tacho'][1]", "['bh_pid_output'][0]", "['bh_pid_output'][1]", "['bh_error'][0]", "['bh_error'][1]", "['bh_integral'][0]", "['bh_integral'][1]", "['bh_pwm'][0]", "['bh_pwm'][1]"],


    #### BallHandlers
    "BALLHANDLERS_BALL_POSSESSION": [""],

    # bh_left.angle, bh_right.angle, bh_left.velocity, bh_right.velocity
    "BALLHANDLERS_FEEDBACK": ["[0]", "[1]", "[2]", "[3]"],

    # enabled, bh_left.angle, bh_right.angle, bh_left.velocity, bh_right.velocity
    "BALLHANDLERS_MOTOR_SETPOINT": ["[0]", "[1][0]", "[1][1]", "[1][2]", "[1][3]"], 

    # ball_possession, bh_left.angle_fraction, bh_left.enabled, bh_left.lifted, bh_left.velocity_setpoint, bh_right.angle_fraction, bh_right.enabled, bh_right.lifted, bh_right.velocity_setpoint
    "DIAG_BALLHANDLING": ["['ballPossession']", "['left']['angleFraction']", "['left']['enabled']", "['left']['lifted']", "['left']['velocitySetpoint']", "['right']['angleFraction']", "['right']['enabled']", "['right']['lifted']", "['right']['velocitySetpoint']"],


    #### Setpoint Generator (SPG)
    # spg.currentPosition.x,y,Rz
    # spg.currentVelocity.x,y,Rz
    # spg.maxVelocity.x,y,Rz
    # spg.maxAcceleration.x,y,Rz
    # spg.targetPosition.x,y,Rz
    # spg.targetVelocity.x,y,Rz
    # spg.newPosition.x,y,Rz
    # spg.newVelocity.x,y,Rz
    "DIAG_VELOCITYCONTROL": ["['currentPosition'][0]", "['currentPosition'][1]", "['currentPosition'][2]", "['currentVelocity'][0]", "['currentVelocity'][1]", "['currentVelocity'][2]", "['maxVelocity'][0]", "['maxVelocity'][1]", "['maxVelocity'][2]", "['maxAcceleration'][0]", "['maxAcceleration'][1]", "['maxAcceleration'][2]", "['targetPosition'][0]", "['targetPosition'][1]", "['targetPosition'][2]", "['targetVelocity'][0]", "['targetVelocity'][1]", "['targetVelocity'][2]", "['newPosition'][0]", "['newPosition'][1]", "['newPosition'][2]", "['newVelocity'][0]", "['newVelocity'][1]", "['newVelocity'][2]"],
}

plotDataMapping[PlotData.PLOT_BH] = \
{
    "BALLHANDLERS_BALL_POSSESSION": [""],

    # bh_left.angle, bh_right.angle, bh_left.velocity, bh_right.velocity
    "BALLHANDLERS_FEEDBACK": ["[0]", "[1]", "[2]", "[3]"],

    # enabled, bh_left.angle, bh_right.angle, bh_left.velocity, bh_right.velocity
    "BALLHANDLERS_MOTOR_SETPOINT": ["[0]", "[1][0]", "[1][1]", "[1][2]", "[1][3]"], 

    # ball_possession, bh_left.angle_fraction, bh_left.enabled, bh_left.lifted, bh_left.velocity_setpoint, bh_right.angle_fraction, bh_right.enabled, bh_right.lifted, bh_right.velocity_setpoint
    "DIAG_BALLHANDLING": ["['ballPossession']", "['left']['angleFraction']", "['left']['enabled']", "['left']['lifted']", "['left']['velocitySetpoint']", "['right']['angleFraction']", "['right']['enabled']", "['right']['lifted']", "['right']['velocitySetpoint']"],

    # bh_left.angle, bh_right.angle, bh_left.tacho_zero, bh_right.tacho_zero, bh_left.tacho, bh_right.tacho, bh_left.pid_output, bh_right.pid_output, bh_left.error, bh_right.error, bh_left.integral, bh_right.integral, bh_left.pwm, bh_right.pwm
    "DIAG_PERIPHERALSINTERFACE": ["['bh_angle'][0]", "['bh_angle'][1]", "['bh_tacho_zero'][0]", "['bh_tacho_zero'][1]", "['bh_tacho'][0]", "['bh_tacho'][1]", "['bh_pid_output'][0]", "['bh_pid_output'][1]", "['bh_error'][0]", "['bh_error'][1]", "['bh_integral'][0]", "['bh_integral'][1]", "['bh_pwm'][0]", "['bh_pwm'][1]"]
}

plotDataMapping[PlotData.PLOT_LATENCY] = \
{
    #### Robot PosVel Setpoint
    # timestamp, robot.vx.setpoint, robot.vy.setpoint, robot.vRz.setpoint
    "ROBOT_POSVEL_SETPOINT": ["timestamp", "[2][0]", "[2][1]", "[2][2]"],

    ### Robot Displacement Feedback
    # timestamp, robot.x.displacement, robot.y.displacement, robot.Rz.displacement
    "ROBOT_DISPLACEMENT_FEEDBACK": ["timestamp", "[0]", "[1]", "[2]"],

    ### Localization Candidates
    # timestamp, loc_candidate.timestamp.seconds, loc_candidate.x, loc_candidate.y, loc_candidate.Rz
    "LOCALIZATION_CANDIDATES": ["timestamp", "[0][1]", "[0][2]", "[0][3]", "[0][4]"]
}
