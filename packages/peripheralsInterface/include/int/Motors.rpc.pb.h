// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
// THIS FILE IS GENERATED, DO NOT EDIT!

#ifndef MOTORS_
#define MOTORS_

#include <string>

#include "Rpc.hpp"
#include "Motors.pb.h"

namespace Motors
{
class Client : public Rpc::Client
{
public:
    Client(const std::string host, uint16_t port);
    ~Client();

    ::motors::Empty enable_ballhandlers(const ::motors::Empty& request);
    ::motors::Empty disable_ballhandlers(const ::motors::Empty& request);
    ::motors::Status is_ballhandlers_enabled(const ::motors::Empty& request);
    ::motors::BallhandlerAngles get_ballhandler_angles(const ::motors::Empty& request);
    ::motors::BallhandlerAngles get_ballhandler_angle_setpoints(const ::motors::Empty& request);
    ::motors::Empty set_ballhandler_angle_setpoints(const ::motors::BallhandlerAngles& request);
    ::motors::MotorPID get_ballhandlers_pid(const ::motors::Empty& request);
    ::motors::Empty set_ballhandlers_pid(const ::motors::MotorPID& request);
    ::motors::Empty enable_drive_motors(const ::motors::Empty& request);
    ::motors::Empty disable_drive_motors(const ::motors::Empty& request);
    ::motors::Status is_drive_motors_enabled(const ::motors::Empty& request);
    ::motors::RobotVector get_robot_velocity(const ::motors::Empty& request);
    ::motors::RobotVector get_robot_position(const ::motors::Empty& request);
    ::motors::RobotVector get_robot_velocity_setpoint(const ::motors::Empty& request);
    ::motors::Empty set_robot_velocity_setpoint(const ::motors::RobotVector& request);
    ::motors::MotorPID get_drive_motors_pid(const ::motors::Empty& request);
    ::motors::Empty set_drive_motors_pid(const ::motors::MotorPID& request);
};

class Server : public Rpc::Server
{
public:
    Server(uint16_t port);
    ~Server();

protected:
    virtual void enable_ballhandlers(const ::motors::Empty& request, ::motors::Empty& response) = 0;
    virtual void disable_ballhandlers(const ::motors::Empty& request, ::motors::Empty& response) = 0;
    virtual void is_ballhandlers_enabled(const ::motors::Empty& request, ::motors::Status& response) = 0;
    virtual void get_ballhandler_angles(const ::motors::Empty& request, ::motors::BallhandlerAngles& response) = 0;
    virtual void get_ballhandler_angle_setpoints(const ::motors::Empty& request, ::motors::BallhandlerAngles& response) = 0;
    virtual void set_ballhandler_angle_setpoints(const ::motors::BallhandlerAngles& request, ::motors::Empty& response) = 0;
    virtual void get_ballhandlers_pid(const ::motors::Empty& request, ::motors::MotorPID& response) = 0;
    virtual void set_ballhandlers_pid(const ::motors::MotorPID& request, ::motors::Empty& response) = 0;
    virtual void enable_drive_motors(const ::motors::Empty& request, ::motors::Empty& response) = 0;
    virtual void disable_drive_motors(const ::motors::Empty& request, ::motors::Empty& response) = 0;
    virtual void is_drive_motors_enabled(const ::motors::Empty& request, ::motors::Status& response) = 0;
    virtual void get_robot_velocity(const ::motors::Empty& request, ::motors::RobotVector& response) = 0;
    virtual void get_robot_position(const ::motors::Empty& request, ::motors::RobotVector& response) = 0;
    virtual void get_robot_velocity_setpoint(const ::motors::Empty& request, ::motors::RobotVector& response) = 0;
    virtual void set_robot_velocity_setpoint(const ::motors::RobotVector& request, ::motors::Empty& response) = 0;
    virtual void get_drive_motors_pid(const ::motors::Empty& request, ::motors::MotorPID& response) = 0;
    virtual void set_drive_motors_pid(const ::motors::MotorPID& request, ::motors::Empty& response) = 0;

private:
    ::google::protobuf::Message* get_request_prototype(uint8_t method);
    ::google::protobuf::Message* get_response_prototype(uint8_t method);
    void call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response);
};
}

#endif // MOTORS_
