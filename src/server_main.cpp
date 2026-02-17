#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <grpcpp/grpcpp.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>

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
    explicit DroneControlService(mavsdk::Action& action) : _action(action) {}

    grpc::Status Enqueue(grpc::ServerContext*,
                         const drone::Command* req,
                         drone::CommandAck* reply) override
    {
        reply->set_id(req->id());

        // For now: execute immediately (no queue yet)
        if (req->has_hover()) {
            const double seconds = req->hover().seconds();
            std::cout << "[API] Hover request: " << seconds << "s\n";

            auto r = _action.arm();
            if (r != mavsdk::Action::Result::Success) {
                reply->set_accepted(false);
                reply->set_message(std::string("Arm failed: ") + action_result_to_string(r));
                return grpc::Status::OK;
            }

            r = _action.takeoff();
            if (r != mavsdk::Action::Result::Success) {
                reply->set_accepted(false);
                reply->set_message(std::string("Takeoff failed: ") + action_result_to_string(r));
                return grpc::Status::OK;
            }

            // Give it a moment to climb, then wait requested time
            std::this_thread::sleep_for(3s);
            std::this_thread::sleep_for(std::chrono::duration<double>(seconds));

            r = _action.land();
            if (r != mavsdk::Action::Result::Success) {
                reply->set_accepted(false);
                reply->set_message(std::string("Land failed: ") + action_result_to_string(r));
                return grpc::Status::OK;
            }

            reply->set_accepted(true);
            reply->set_message("Executed: arm -> takeoff -> wait -> land");
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
        // “Stop now” for this simple demo: command HOLD (loiter)
        auto r = _action.hold();
        reply->set_ok(r == mavsdk::Action::Result::Success);
        reply->set_message(std::string("Hold result: ") + action_result_to_string(r));
        std::cout << "[API] StopNow -> hold()\n";
        return grpc::Status::OK;
    }

private:
    mavsdk::Action& _action;
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
    action.set_takeoff_altitude(2.0f);

    // 2) Start gRPC server
    std::string addr = "0.0.0.0:60051";
    DroneControlService service(action);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    auto server = builder.BuildAndStart();
    std::cout << "Drone API server listening on " << addr << "\n";
    server->Wait();
    return 0;
}
