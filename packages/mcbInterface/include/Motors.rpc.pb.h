 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
