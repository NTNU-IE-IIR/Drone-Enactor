// src/server_main.cpp

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <grpcpp/grpcpp.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>

#include "flight_controller.h"
#include "drone_control_service.h"

using namespace std::chrono_literals;

static std::shared_ptr<mavsdk::System> wait_for_autopilot(mavsdk::Mavsdk& mavsdk,
                                                         std::chrono::seconds timeout)
{
    const auto start = std::chrono::steady_clock::now();

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

int main()
{
    mavsdk::Mavsdk mavsdk{
        mavsdk::Mavsdk::Configuration{
            mavsdk::ComponentType::GroundStation
        }
    };

    const std::string conn = "udpin://0.0.0.0:14550";

    auto result = mavsdk.add_any_connection(conn);
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    std::cout << "Waiting for PX4...\n";
    auto system = wait_for_autopilot(mavsdk, 20s);

    if (!system) {
        std::cerr << "No autopilot found\n";
        return 1;
    }

    std::cout << "Autopilot found\n";

    FlightController controller(system);

    DroneControlService service(controller);

    const std::string addr = "0.0.0.0:60051";

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());

    if (!server) {
        std::cerr << "Failed to start server\n";
        return 1;
    }

    std::cout << "Drone API listening on " << addr << "\n";

    server->Wait();
    return 0;
}