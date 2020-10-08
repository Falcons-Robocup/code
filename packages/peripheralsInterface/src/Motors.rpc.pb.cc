// THIS FILE IS GENERATED, DO NOT EDIT!
#include "int/Motors.rpc.pb.h"

namespace Motors
{
Client::Client(const std::string host, uint16_t port) :
    Rpc::Client(host, port)
{
}

Client::~Client()
{
}

::motors::Empty Client::enable_ballhandlers(const ::motors::Empty& request)
{
    ::motors::Empty response;
    call(0, request, response);
    return response;
}

::motors::Empty Client::disable_ballhandlers(const ::motors::Empty& request)
{
    ::motors::Empty response;
    call(1, request, response);
    return response;
}

::motors::Status Client::is_ballhandlers_enabled(const ::motors::Empty& request)
{
    ::motors::Status response;
    call(2, request, response);
    return response;
}

::motors::BallhandlerAngles Client::get_ballhandler_angles(const ::motors::Empty& request)
{
    ::motors::BallhandlerAngles response;
    call(3, request, response);
    return response;
}

::motors::BallhandlerAngles Client::get_ballhandler_angle_setpoints(const ::motors::Empty& request)
{
    ::motors::BallhandlerAngles response;
    call(4, request, response);
    return response;
}

::motors::Empty Client::set_ballhandler_angle_setpoints(const ::motors::BallhandlerAngles& request)
{
    ::motors::Empty response;
    call(5, request, response);
    return response;
}

::motors::MotorPID Client::get_ballhandlers_pid(const ::motors::Empty& request)
{
    ::motors::MotorPID response;
    call(6, request, response);
    return response;
}

::motors::Empty Client::set_ballhandlers_pid(const ::motors::MotorPID& request)
{
    ::motors::Empty response;
    call(7, request, response);
    return response;
}

::motors::Empty Client::enable_drive_motors(const ::motors::Empty& request)
{
    ::motors::Empty response;
    call(8, request, response);
    return response;
}

::motors::Empty Client::disable_drive_motors(const ::motors::Empty& request)
{
    ::motors::Empty response;
    call(9, request, response);
    return response;
}

::motors::Status Client::is_drive_motors_enabled(const ::motors::Empty& request)
{
    ::motors::Status response;
    call(10, request, response);
    return response;
}

::motors::RobotVector Client::get_robot_velocity(const ::motors::Empty& request)
{
    ::motors::RobotVector response;
    call(11, request, response);
    return response;
}

::motors::RobotVector Client::get_robot_position(const ::motors::Empty& request)
{
    ::motors::RobotVector response;
    call(12, request, response);
    return response;
}

::motors::RobotVector Client::get_robot_velocity_setpoint(const ::motors::Empty& request)
{
    ::motors::RobotVector response;
    call(13, request, response);
    return response;
}

::motors::Empty Client::set_robot_velocity_setpoint(const ::motors::RobotVector& request)
{
    ::motors::Empty response;
    call(14, request, response);
    return response;
}

::motors::MotorPID Client::get_drive_motors_pid(const ::motors::Empty& request)
{
    ::motors::MotorPID response;
    call(15, request, response);
    return response;
}

::motors::Empty Client::set_drive_motors_pid(const ::motors::MotorPID& request)
{
    ::motors::Empty response;
    call(16, request, response);
    return response;
}

Server::Server(uint16_t port) :
    Rpc::Server(port)
{
}

Server::~Server()
{
}

::google::protobuf::Message* Server::get_request_prototype(uint8_t method)
{
    ::google::protobuf::Message* request_prototype;
    switch (method)
    {
    case 0:
        request_prototype = new ::motors::Empty();
        break;
    case 1:
        request_prototype = new ::motors::Empty();
        break;
    case 2:
        request_prototype = new ::motors::Empty();
        break;
    case 3:
        request_prototype = new ::motors::Empty();
        break;
    case 4:
        request_prototype = new ::motors::Empty();
        break;
    case 5:
        request_prototype = new ::motors::BallhandlerAngles();
        break;
    case 6:
        request_prototype = new ::motors::Empty();
        break;
    case 7:
        request_prototype = new ::motors::MotorPID();
        break;
    case 8:
        request_prototype = new ::motors::Empty();
        break;
    case 9:
        request_prototype = new ::motors::Empty();
        break;
    case 10:
        request_prototype = new ::motors::Empty();
        break;
    case 11:
        request_prototype = new ::motors::Empty();
        break;
    case 12:
        request_prototype = new ::motors::Empty();
        break;
    case 13:
        request_prototype = new ::motors::Empty();
        break;
    case 14:
        request_prototype = new ::motors::RobotVector();
        break;
    case 15:
        request_prototype = new ::motors::Empty();
        break;
    case 16:
        request_prototype = new ::motors::MotorPID();
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }

    return request_prototype;
}

::google::protobuf::Message* Server::get_response_prototype(uint8_t method)
{
    ::google::protobuf::Message* response_prototype;
    switch (method)
    {
    case 0:
        response_prototype = new ::motors::Empty();
        break;
    case 1:
        response_prototype = new ::motors::Empty();
        break;
    case 2:
        response_prototype = new ::motors::Status();
        break;
    case 3:
        response_prototype = new ::motors::BallhandlerAngles();
        break;
    case 4:
        response_prototype = new ::motors::BallhandlerAngles();
        break;
    case 5:
        response_prototype = new ::motors::Empty();
        break;
    case 6:
        response_prototype = new ::motors::MotorPID();
        break;
    case 7:
        response_prototype = new ::motors::Empty();
        break;
    case 8:
        response_prototype = new ::motors::Empty();
        break;
    case 9:
        response_prototype = new ::motors::Empty();
        break;
    case 10:
        response_prototype = new ::motors::Status();
        break;
    case 11:
        response_prototype = new ::motors::RobotVector();
        break;
    case 12:
        response_prototype = new ::motors::RobotVector();
        break;
    case 13:
        response_prototype = new ::motors::RobotVector();
        break;
    case 14:
        response_prototype = new ::motors::Empty();
        break;
    case 15:
        response_prototype = new ::motors::MotorPID();
        break;
    case 16:
        response_prototype = new ::motors::Empty();
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }

    return response_prototype;
}

void Server::call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response)
{
    switch (method)
    {
    case 0:
        enable_ballhandlers(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 1:
        disable_ballhandlers(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 2:
        is_ballhandlers_enabled(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Status&>(response));
        break;
    case 3:
        get_ballhandler_angles(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::BallhandlerAngles&>(response));
        break;
    case 4:
        get_ballhandler_angle_setpoints(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::BallhandlerAngles&>(response));
        break;
    case 5:
        set_ballhandler_angle_setpoints(::google::protobuf::down_cast<const ::motors::BallhandlerAngles&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 6:
        get_ballhandlers_pid(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::MotorPID&>(response));
        break;
    case 7:
        set_ballhandlers_pid(::google::protobuf::down_cast<const ::motors::MotorPID&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 8:
        enable_drive_motors(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 9:
        disable_drive_motors(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 10:
        is_drive_motors_enabled(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::Status&>(response));
        break;
    case 11:
        get_robot_velocity(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::RobotVector&>(response));
        break;
    case 12:
        get_robot_position(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::RobotVector&>(response));
        break;
    case 13:
        get_robot_velocity_setpoint(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::RobotVector&>(response));
        break;
    case 14:
        set_robot_velocity_setpoint(::google::protobuf::down_cast<const ::motors::RobotVector&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    case 15:
        get_drive_motors_pid(::google::protobuf::down_cast<const ::motors::Empty&>(request), ::google::protobuf::down_cast<::motors::MotorPID&>(response));
        break;
    case 16:
        set_drive_motors_pid(::google::protobuf::down_cast<const ::motors::MotorPID&>(request), ::google::protobuf::down_cast<::motors::Empty&>(response));
        break;
    default:
        throw(std::runtime_error("Unknown method " + std::to_string(method)));
        break;
    }
}
}
