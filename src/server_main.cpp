#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <grpcpp/grpcpp.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include "drone_control.grpc.pb.h"

using namespace std::chrono_literals;

static const char* action_result_to_string(mavsdk::Action::Result r)
{
    using R = mavsdk::Action::Result;
    switch (r) {
        case R::Success: return "Success";
        case R::NoSystem: return "NoSystem";
        case R::ConnectionError: return "ConnectionError";
        case R::Busy: return "Busy";
        case R::CommandDenied: return "CommandDenied";
        case R::CommandDeniedLandedStateUnknown: return "CommandDeniedLandedStateUnknown";
        case R::CommandDeniedNotLanded: return "CommandDeniedNotLanded";
        case R::Timeout: return "Timeout";
        case R::VtolTransitionSupportUnknown: return "VtolTransitionSupportUnknown";
        case R::NoVtolTransitionSupport: return "NoVtolTransitionSupport";
        case R::ParameterError: return "ParameterError";
        case R::Unknown: return "Unknown";
        default: return "Unmapped";
    }
}
static const char* connection_result_to_string(mavsdk::ConnectionResult r)
{
    using R = mavsdk::ConnectionResult;
    switch (r) {
        case R::Success: return "Success";
        case R::Timeout: return "Timeout";
        case R::SocketError: return "SocketError";
        case R::BindError: return "BindError";
        case R::ConnectionError: return "ConnectionError";
        default: return "Unmapped";
    }
}



static std::shared_ptr<mavsdk::System> wait_for_autopilot(mavsdk::Mavsdk& mavsdk, std::chrono::seconds timeout)
{
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < timeout) {
        for (auto& sys : mavsdk.systems()) {
            if (sys && sys->has_autopilot()) {
                return sys;
            }
        }
        std::this_thread::sleep_for(200ms);
    }
    return {};
}

class DroneControlService final : public drone::DroneControl::Service {
public:
    explicit DroneControlService(mavsdk::Action& action, mavsdk::Telemetry& telemetry, mavsdk::System& system)
                                : _action(action), _telemetry(telemetry), _system(system) {}

private: mavsdk::Action& _action;
    mavsdk::Telemetry& _telemetry;
    mavsdk::System& _system;


    grpc::Status Enqueue(grpc::ServerContext*,
                         const drone::Command* req,
                         drone::CommandAck* reply) override
    {
        reply->set_id(req->id());

        if (req->has_hover()) {
            const double seconds = req->hover().seconds();
            std::cout << "[API] Hover request: " << seconds << "s\n";

            if (!ensure_airborn_or_reply(reply)) {
                return grpc::Status::OK;
            }

            std::this_thread::sleep_for(3s);
            std::this_thread::sleep_for(std::chrono::duration<double>(seconds));

            auto r = _action.land();
            if (r != mavsdk::Action::Result::Success) {
                reply->set_accepted(false);
                reply->set_message(std::string("Land failed: ") + action_result_to_string(r));
                return grpc::Status::OK;
            }

            reply->set_accepted(true);
            reply->set_message("Executed: arm -> takeoff -> wait -> land");
            return grpc::Status::OK;
        }
        if (req->has_flyforward()) {
            mavsdk::Offboard offboard {_system};
            if (!ensure_airborn_or_reply(reply)) {
                return grpc::Status::OK;
            }
            if (!wait_until_ready_for_offboard(reply)) {
                return grpc::Status::OK;
            }
            const double seconds = req -> flyforward().seconds();
            const double velocity = req -> flyforward().velocity();
            std::cout << "[API] Fly forward request for " << seconds
            << "s seconds and velocity " << velocity << "m/s\n";

            grpc::Status value;
            mavsdk::Offboard::VelocityBodyYawspeed cmd{};
            cmd.forward_m_s = static_cast<float>(velocity);
            cmd.down_m_s = 0.0;
            cmd.right_m_s = 0.0;
            cmd.yawspeed_deg_s = 0.0;

            offboard.set_velocity_body(cmd);

            for (int i = 0; i < 20; ++i) {
                offboard.set_velocity_body(cmd);
                std::this_thread::sleep_for(50ms);
            }
            auto r = offboard.start();
            std::cout << "Flight mode: " << static_cast<int>(_telemetry.flight_mode()) << "\n";
            if (r != mavsdk::Offboard::Result::Success) {
                reply -> set_accepted(false);
                reply -> set_message("Offboard failed");
                return grpc::Status::OK;
            }
            const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
            while (std::chrono::steady_clock::now() < end) {
                offboard.set_velocity_body(cmd);
                std::this_thread::sleep_for(100ms);
            }

            cmd.forward_m_s = 0.0;
            offboard.set_velocity_body(cmd);

            offboard.stop();
            reply -> set_accepted(true);
            reply -> set_message("Flyforward executed");
            return grpc::Status::OK;
        }
        reply->set_accepted(false);
        reply->set_message("Unsupported command payload");
        return grpc::Status::OK;
    }


    grpc::Status StopNow(grpc::ServerContext*,
                         const drone::StopRequest*,
                         drone::StopReply* reply) override
    {
        auto r = _action.hold();
        reply->set_ok(r == mavsdk::Action::Result::Success);
        reply->set_message(std::string("Hold result: ") + action_result_to_string(r));
        std::cout << "[API] StopNow -> hold()\n";
        return grpc::Status::OK;
    }

    bool ensure_airborn_or_reply(drone::CommandAck* reply) {

        const auto start = std::chrono::steady_clock::now();
        while (!_telemetry.health_all_ok()) {
            if (std::chrono::steady_clock::now() - start < 10s) {
                reply -> set_accepted(false);
                reply -> set_message("Preflight NOT OK, health_status == false after 10 seconds");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }
        if (_telemetry.armed() != true) {
            auto r = _action.arm();
            if (mavsdk::Action::Result::Success != r) {
                reply -> set_accepted(false);
                reply -> set_message("Arm failed");
                return false;
            }
        }

        const auto arm_start = std::chrono::steady_clock::now();
        while (!_telemetry.health_all_ok()) {
            if (std::chrono::steady_clock::now() - start < 5s) {
                reply -> set_accepted(false);
                reply -> set_message("Arming failed, health_status = false after 5 seconds");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }

        if (_telemetry.in_air() != true) {
            auto r = _action.takeoff();
            if (r != mavsdk::Action::Result::Success) {
                reply -> set_accepted(false);
                reply -> set_message("Takeoff failed");
                return false;
            }
        }
        const auto to_start = std::chrono::steady_clock::now();
        while (!_telemetry.health_all_ok()) {
            if (std::chrono::steady_clock::now() - start < 5s) {
                reply -> set_accepted(false);
                reply -> set_message("Takeoff failed, health_status = false after 5 seconds");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }
        return true;
    }
    bool wait_until_ready_for_offboard(drone::CommandAck* reply)
    {
        // Wait until we're in air
        const auto t0 = std::chrono::steady_clock::now();
        while (!_telemetry.in_air()) {
            if (std::chrono::steady_clock::now() - t0 > 10s) {
                reply->set_accepted(false);
                reply->set_message("Not in air after 10s");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }

        // Wait until TAKEOFF mode is finished (PX4 often stays in Takeoff mode briefly)
        const auto t1 = std::chrono::steady_clock::now();
        while (_telemetry.flight_mode() == mavsdk::Telemetry::FlightMode::Takeoff) {
            if (std::chrono::steady_clock::now() - t1 > 10s) {
                reply->set_accepted(false);
                reply->set_message("Still in Takeoff flight mode after 10s");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }

        return true;
    }
};


int main()
{
    // 1) Connect MAVSDK to PX4 SITL (usual default)
    mavsdk::Mavsdk mavsdk{
        mavsdk::Mavsdk::Configuration{
            mavsdk::ComponentType::GroundStation
        }
    };
    const std::string conn = "udpin://0.0.0.0:14540";

    auto conn_result = mavsdk.add_any_connection(conn);
    if (conn_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result_to_string(conn_result) << "\n";
        return 1;
    }
    std::cout << "Waiting for PX4 autopilot on " << conn << "...\n";

    auto system = wait_for_autopilot(mavsdk, 10s);
    if (!system) {
        std::cerr << "No autopilot found (is SITL running and publishing on 14540?)\n";
        return 1;
    }
    std::cout << "Autopilot found.\n";

    mavsdk::Action action{system};
    mavsdk::Telemetry telemetry{system};
    action.set_takeoff_altitude(2.0f);

    std::string addr = "0.0.0.0:60051";
    DroneControlService service(action, telemetry, *system);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    auto server = builder.BuildAndStart();
    std::cout << "Drone API server listening on " << addr << "\n";
    server->Wait();
    return 0;
}
