 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <chrono>
#include <exception>
#include <string>
#include <thread>

#include <zmq.hpp>

#include "cDiagnostics.hpp"
#include "tracing.hpp"
//#include "falconsCommon.hpp"

#include "RTDBInputAdapter.hpp"
#include "RTDBOutputAdapter.hpp"

#include "Motors.rpc.pb.h"
#include "Kicker.rpc.pb.h"

using namespace std::chrono;
using namespace std::this_thread;

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
    void run_motors_client();
    void run_subscriber();

    void start_software();
    void stop_software();

    // RTDB adapters
    RTDBInputAdapter input_adapter;
    RTDBOutputAdapter output_adapter;

    // Threads
    thread kicker_client_thread;
    thread motors_client_thread;
    thread subscriber_thread;

    // Servers
    Motors::Client motors;
    Kicker::Client kicker;

    // Publisher
    zmq::context_t zmq_context;
    zmq::socket_t zmq_subscriber;

    bool software_on;
};

McbInterface::McbInterface() :
    motors("10.42.0.2", 10001), kicker("10.42.0.2", 10002), zmq_context(1), zmq_subscriber(zmq_context, ZMQ_SUB),
        software_on(false)
{
}

McbInterface::~McbInterface()
{
}

void McbInterface::run()
{
    zmq_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // filter
    zmq_subscriber.connect("tcp://10.42.0.2:10000");

    kicker_client_thread = thread(&McbInterface::run_kicker_client, this);
    motors_client_thread = thread(&McbInterface::run_motors_client, this);
    subscriber_thread = thread(&McbInterface::run_subscriber, this);

    kicker_client_thread.join();
    motors_client_thread.join();
    subscriber_thread.join();

    TRACE("ZeroMQ subscription correct.");
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
                ::kicker::Height request;
                request.set_value(0.0);
                ::kicker::Empty response = kicker.set_height(request);
                break;
            }
            case KickerSetpoint::Type::SetHeight: {
                std::stringstream str;
                str << "height=" << setpoint.value;
                TRACE_SCOPE("SET_KICKER_HEIGHT", str.str().c_str());

                ::kicker::Height request;
                request.set_value(setpoint.value);
                ::kicker::Empty response = kicker.set_height(request);
                break;
            }
            case KickerSetpoint::Type::Shoot: {
                std::stringstream str;
                str << "power=" << setpoint.value;
                TRACE_SCOPE("SHOOT_KICKER", str.str().c_str());

                // only shoot if power > 0
                if (setpoint.value > 0) {
                    ::kicker::ShootPower request;
                    request.set_value(setpoint.value);
                    ::kicker::Empty response = kicker.shoot(request);
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

void McbInterface::run_motors_client()
{
    TRACE("Starting motors server.");

    while (true) {
        // Calculate the time when the next iteration should be started.
        system_clock::time_point end_time = system_clock::now() + milliseconds(10);

        try {
            ::motors::RobotVector velocity_setpoint;
            try {
                velocity_setpoint = input_adapter.getRobotVelocitySetpoint();
            }
            catch (exception& e) {
                velocity_setpoint.set_x(0);
                velocity_setpoint.set_y(0);
                velocity_setpoint.set_phi(0);
            }
            motors.set_robot_velocity_setpoint(velocity_setpoint);

            if (input_adapter.getBallhandlersOn()) {
                motors.enable_ballhandlers(motors::Empty());
            }
            else {
                motors.disable_ballhandlers(motors::Empty());
            }

            motors::BallhandlerAngles angle_setpoints = input_adapter.getBallhandlerAngleSetpoints();
            motors.set_ballhandler_angle_setpoints(angle_setpoints);
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
        zmq_subscriber.recv(&message);

        ::motors::Publish publish;
        publish.ParseFromArray(message.data(), message.size());

        output_adapter.setBallHandlerAngles(publish.ballhandler_angles());
        output_adapter.setInPlayStatus(publish.inplay().enabled());
        output_adapter.setRobotPosition(publish.robot_position());
        output_adapter.setRobotVelocity(publish.robot_velocity());

        if (publish.software_on().enabled() != software_on) {
            try {
                if (publish.software_on().enabled()) {
                    start_software();
                    TRACE_INFO("Software On switch triggered, software is starting.");
                }
                else {
                    stop_software();
                    TRACE_INFO("Software On switch triggered, software is shutting down.");
                }

                software_on = publish.software_on().enabled();
            }
            catch (exception &e) {
                TRACE_ERROR("Failed to stop or start software: %s", e.what());
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
