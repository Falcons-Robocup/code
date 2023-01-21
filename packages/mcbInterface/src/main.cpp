// Copyright 2019-2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <exception>
#include <string>
#include <thread>

#include <zmq.hpp>

#include "cDiagnostics.hpp"
#include "tracing.hpp"
//#include "falconsCommon.hpp"

#include "ConfigRTDBAdapter.hpp"
#include "RTDBInputAdapter.hpp"
#include "RTDBOutputAdapter.hpp"

#include "publish.pb.h"
#include "motion.rpc.pb.h"
#include "kicker.rpc.pb.h"
#include "motors.rpc.pb.h"

using namespace std::chrono;
using namespace std::this_thread;
using namespace google;

using std::exception;
using std::runtime_error;
using std::string;
using std::thread;
using std::to_string;

class McbInterface
{
public:
    McbInterface();
    ~McbInterface();

    void run();

private:
    void run_kicker_client();
    void run_motion_client();
    void run_subscriber();

    void start_software();
    void stop_software();

    void update_motors_configuration();

    // RTDB adapters
    RTDBInputAdapter input_adapter;
    RTDBOutputAdapter output_adapter;
    ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_MOTORS>* _configAdapterMotors;

    // Threads
    thread kicker_client_thread;
    thread motion_client_thread;
    thread subscriber_thread;

    // Servers
    Motion::Client motion;
    Kicker::Client kicker;
    Motors::Client motors;

    // Publisher
    zmq::context_t zmq_context;
    zmq::socket_t zmq_subscriber;

    bool software_on;
    bool new_motors_configuration_available;
};

McbInterface::McbInterface() :
    motion("10.42.0.2", 10001), kicker("10.42.0.2", 10002), motors("10.42.0.2", 10003), zmq_context(1), zmq_subscriber(zmq_context, ZMQ_SUB),
        software_on(false), new_motors_configuration_available(false)
{
}

McbInterface::~McbInterface()
{
}

void McbInterface::run()
{
    INIT_TRACE("mcbInterface");

    zmq_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // filter
    zmq_subscriber.connect("tcp://10.42.0.2:10000");

    // config adapter Motors
    _configAdapterMotors = new ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_MOTORS>(CONFIG_PERIPHERALSINTERFACE_MOTORS);
    std::string configFileMotors = determineConfig("Motors");
    _configAdapterMotors->loadYAML(configFileMotors);
    _configAdapterMotors->setConfigUpdateCallback( std::bind(&McbInterface::update_motors_configuration, this) );
    update_motors_configuration();

    kicker_client_thread = thread(&McbInterface::run_kicker_client, this);
    motion_client_thread = thread(&McbInterface::run_motion_client, this);
    subscriber_thread = thread(&McbInterface::run_subscriber, this);

    kicker_client_thread.join();
    motion_client_thread.join();
    subscriber_thread.join();

    TRACE("ZeroMQ subscription correct.");
}

void McbInterface::update_motors_configuration()
{
    new_motors_configuration_available = true;
}

void McbInterface::run_kicker_client()
{
    while (true) {
        try {
            input_adapter.waitForKickerSetpoint();

            KickerSetpoint setpoint = input_adapter.getKickerSetpoint();

            switch (setpoint.type) {
            case KickerSetpoint::Type::Home: {
                TRACE_SCOPE("HOME_KICKER", "");
                protobuf::FloatValue request;
                request.set_value(0.0);
                protobuf::Empty response = kicker.set_height(request);
                break;
            }
            case KickerSetpoint::Type::SetHeight: {
                std::stringstream str;
                str << "height=" << setpoint.value;
                TRACE_SCOPE("SET_KICKER_HEIGHT", str.str().c_str());

                protobuf::FloatValue request;
                request.set_value(setpoint.value);
                protobuf::Empty response = kicker.set_height(request);
                break;
            }
            case KickerSetpoint::Type::Shoot: {
                std::stringstream str;
                str << "power=" << setpoint.value;
                TRACE_SCOPE("SHOOT_KICKER", str.str().c_str());

                // only shoot if power > 0
                if (setpoint.value > 0) {
                    protobuf::FloatValue request;
                    request.set_value(setpoint.value);
                    protobuf::Empty response = kicker.shoot(request);
                }
                break;
            }
            }
        }
        catch (exception& e) {
            TRACE("Failed to update kicker.");
        }
    }
}

void McbInterface::run_motion_client()
{
    TRACE("Starting motion server.");

    while (true) {
        // Calculate the time when the next iteration should be started.
        system_clock::time_point end_time = system_clock::now() + milliseconds(10);

        try {
            // Update motors configuration if needed
            if (new_motors_configuration_available)
            {
               T_CONFIG_PERIPHERALSINTERFACE_MOTORS configMotors;
               _configAdapterMotors->get(configMotors);

               Motors::VelocityControlParameterSet velocity_param_set;
               velocity_param_set.set_p_gain(configMotors.Kp);
               velocity_param_set.set_i_gain(configMotors.Ki);

               // apply the p_gain and i_gain to every motor
               velocity_param_set.set_motor(Motors::MotorEnum::DRIVE_MOTOR_LEFT);
               motors.set_velocity_pid_settings(velocity_param_set);
               velocity_param_set.set_motor(Motors::MotorEnum::DRIVE_MOTOR_RIGHT);
               motors.set_velocity_pid_settings(velocity_param_set);
               velocity_param_set.set_motor(Motors::MotorEnum::DRIVE_MOTOR_REAR);
               motors.set_velocity_pid_settings(velocity_param_set);

               new_motors_configuration_available = false;
            }

            ::Motion::RobotVector velocity_setpoint;
            bool has_robot_velocity_setpoint = false;
            try {
                velocity_setpoint = input_adapter.getRobotVelocitySetpoint();
                has_robot_velocity_setpoint = true;
            }
            catch (exception& e) {
                TRACE("Failed to get RobotVelocitySetpoint: %s", e.what());
                velocity_setpoint.set_x(0);
                velocity_setpoint.set_y(0);
                velocity_setpoint.set_phi(0);
            }

            std::ostringstream oss;
            oss << "ROBOT_VELOCITY_SETPOINT=(" << velocity_setpoint.x() << ", " << velocity_setpoint.y() << ", " << velocity_setpoint.phi() << ")";
            TRACE(oss.str().c_str());

            if (has_robot_velocity_setpoint)
            {
                motion.set_robot_velocity_setpoint(velocity_setpoint);
            }

            if (input_adapter.getBallhandlersOn()) {
                motion.enable_ballhandlers(protobuf::Empty());
            }
            else {
                motion.disable_ballhandlers(protobuf::Empty());
            }

            Motion::BallhandlerVector angle_setpoints = input_adapter.getBallhandlerAngleSetpoints();
            motion.set_ballhandler_angle_setpoints(angle_setpoints);
        }
        catch (exception& e) {
            TRACE("Failed to update controller: %s", e.what());
        }

        sleep_until(end_time);

        WRITE_TRACE;
    }
}

void McbInterface::run_subscriber()
{
    TRACE("Starting controller subscriber.");

    while (true) {
        zmq::message_t message;
        zmq::recv_result_t message_result = zmq_subscriber.recv(message);

        {
            ::Publish::Message publish;
            if (message_result)
            {
                publish.ParseFromArray(message.data(), message.size());
            }
            output_adapter.setBallHandlerAngles(publish.ballhandler_angles());
            output_adapter.setInPlayStatus(publish.inplay());
            output_adapter.setRobotPosition(publish.robot_position());
            output_adapter.setRobotVelocity(publish.robot_velocity());

            // diag
            output_adapter.setMotorDiagnosticsOutput(publish.motor_torque(), publish.motor_velocity(), publish.motor_velocity_demand(),publish.motor_position(), publish.motor_current(), publish.motor_current_demand());
            output_adapter.setMotorFeedback(publish.motor_velocity());

            if (publish.software_on() != software_on) {
                try {

                    // First update internal administration because if the start_software() throws an exception,
                    // it will continuously attempt to start software.
                    software_on = publish.software_on();

                    if (publish.software_on()) {
                        start_software();
                        TRACE_INFO("Software switch toggled, software is starting.");
                    }
                    else {
                        stop_software();
                        TRACE_INFO("Software switch toggled, software is shutting down.");
                    }

                }
                catch (exception &e) {
                    TRACE_ERROR("Failed to stop or start software: %s", e.what());
                }
            }
        }
    }
}

// SW start/stop is handled by helper scripts, see package robotControl

void McbInterface::start_software()
{
    TRACE_FUNCTION("");

    int result = system("robotSwStart");
    if (result != 0) {
        throw(runtime_error("robotSwStart returned " + to_string(result)));
    }
}

void McbInterface::stop_software()
{
    TRACE_FUNCTION("");

    int result = system("robotSwStop");
    if (result < 0) {
        throw(runtime_error("robotSwStart returned " + to_string(result)));
    }
}

int main(int argc, char **argv)
{
    TRACE_FUNCTION("");

    McbInterface mcb_interface;

    mcb_interface.run();
}
