#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <grpcpp/grpcpp.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "drone_control_service.h"

using namespace std::chrono_literals;

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
            if (sys && sys->has_autopilot()) return sys;
        }
        std::this_thread::sleep_for(200ms);
    }
    return {};
}

int main()
{
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
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
    DroneControlService service(*system, action, telemetry);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    auto server = builder.BuildAndStart();
    std::cout << "Drone API server listening on " << addr << "\n";
    server->Wait();
    return 0;
}
