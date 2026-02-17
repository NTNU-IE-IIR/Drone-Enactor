#include "drone_control_service.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <mavsdk/plugins/offboard/offboard.h>

#include "flight_helpers.h"

using namespace std::chrono_literals;

DroneControlService::DroneControlService(mavsdk::System& system,
                                         mavsdk::Action& action,
                                         mavsdk::Telemetry& telemetry)
    : _system(system), _action(action), _telemetry(telemetry) {}

grpc::Status DroneControlService::Enqueue(grpc::ServerContext*,
                                         const drone::Command* req,
                                         drone::CommandAck* reply)
{
    reply->set_id(req->id());

    if (req->has_hover()) {
        return handle_hover(req->hover(), reply);
    }

    if (req->has_flyforward()) {
        return handle_flyforward(req->flyforward(), reply);
    }
    if (req->has_turnyaw()) {
        return handle_turn_yaw(req ->turnyaw(),reply);
    }
    if (req->has_land()) {
        return handle_land(req -> land(), reply);
    }

    reply->set_accepted(false);
    reply->set_message("Unsupported command payload");
    return grpc::Status::OK;
}

grpc::Status DroneControlService::handle_hover(const drone::Hover& hover,
                                              drone::CommandAck* reply)
{
    const double seconds = hover.seconds();
    std::cout << "[API] Hover request: " << seconds << "s\n";

    if (!ensure_airborne_or_reply(_action, _telemetry, reply)) {
        return grpc::Status::OK;
    }
    if (!wait_until_ready_for_offboard_or_reply(_telemetry,reply)) {
        return grpc::Status::OK;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));

    auto r = _action.land();
    if (r != mavsdk::Action::Result::Success) {
        reply->set_accepted(false);
        reply->set_message(std::string("Land failed: ") + action_result_to_string(r));
        return grpc::Status::OK;
    }

    reply->set_accepted(true);
    reply->set_message("Executed: ensure_airborne -> wait -> land");
    return grpc::Status::OK;
}
grpc::Status DroneControlService::handle_land(const drone::Land& land, drone::CommandAck* reply) {
    (void) land;

    std::cout << "[API] Landing request";

    if (!ensure_airborne_or_reply(_action, _telemetry, reply)) {
        return grpc::Status::OK;
    }
    if (!wait_until_ready_for_offboard_or_reply(_telemetry,reply)) {
        return grpc::Status::OK;
    }
    auto r = _action.land();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (r != mavsdk::Action::Result::Success) {
        reply -> set_accepted(false);
        reply -> set_message(std::string("The landing did not complete because: ") + action_result_to_string(r));
        return grpc::Status::OK;
    }
    if (_telemetry.in_air() == false) {
        reply -> set_accepted(true);
        reply -> set_message(std::string("The landing proceeeded succesfully"));
        return grpc::Status::OK;
    }
    reply -> set_accepted(false);
    reply -> set_message(std::string("The landing did not complete for uknown reasons"));
    return grpc::Status::OK;
}

grpc::Status DroneControlService::handle_flyforward(const drone::FlyForward& fly,
                                                    drone::CommandAck* reply)
{
    const double seconds = fly.seconds();
    const double velocity = fly.velocity();

    std::cout << "[API] Fly forward request: " << seconds << "seconds and " << velocity << " m/s\n";

    if (!ensure_airborne_or_reply(_action, _telemetry, reply)) {
        return grpc::Status::OK;
    }
    if (!wait_until_ready_for_offboard_or_reply(_telemetry, reply)) {
        return grpc::Status::OK;
    }

    mavsdk::Offboard offboard{_system};

    mavsdk::Offboard::VelocityBodyYawspeed cmd{};
    cmd.forward_m_s = static_cast<float>(velocity);
    cmd.right_m_s = 0.0f;
    cmd.down_m_s = 0.0f;
    cmd.yawspeed_deg_s = 0.0f;

    // Warm up setpoints (1s @ 20Hz)
    for (int i = 0; i < 20; ++i) {
        offboard.set_velocity_body(cmd);
        std::this_thread::sleep_for(50ms);
    }

    auto r = offboard.start();
    if (r != mavsdk::Offboard::Result::Success) {
        reply->set_accepted(false);
        reply->set_message("Offboard start failed");
        return grpc::Status::OK;
    }

    // Stream setpoints while moving (10Hz)
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        offboard.set_velocity_body(cmd);
        std::this_thread::sleep_for(100ms);
    }

    // Stop movement
    cmd.forward_m_s = 0.0f;
    offboard.set_velocity_body(cmd);
    std::this_thread::sleep_for(200ms);
    offboard.stop();

    reply->set_accepted(true);
    reply->set_message("Flyforward executed");
    return grpc::Status::OK;
}

grpc::Status DroneControlService::handle_turn_yaw(const drone::TurnYaw& turn_yaw, drone::CommandAck* reply) {
    const double deg = turn_yaw.deg();
    const double seconds = turn_yaw.seconds();

    std::cout << "[API] Turn yaw request: " << deg << " degrees and " << seconds << " seconds";

    if (!ensure_airborne_or_reply(_action, _telemetry, reply)) {
        return grpc::Status::OK;
    }
    if (!wait_until_ready_for_offboard_or_reply(_telemetry,reply)) {
        return grpc::Status::OK;
    }
    mavsdk::Offboard offboard {_system};

    mavsdk::Offboard::VelocityBodyYawspeed cmd{};

    cmd.down_m_s = 0.0;
    cmd.forward_m_s = 0.0;
    cmd.right_m_s = 0.0;
    cmd.yawspeed_deg_s = static_cast<float>(deg)/static_cast<float>(seconds);

    for (int i = 0; i < 20; ++i) {
        offboard.set_velocity_body(cmd);
        std::this_thread::sleep_for(50ms);
    }

    auto r = offboard.start();
    if (r != mavsdk::Offboard::Result::Success) {
        reply->set_accepted(false);
        reply->set_message("Offboard start failed");
        return grpc::Status::OK;
    }
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        offboard.set_velocity_body(cmd);
        std::this_thread::sleep_for(100ms);
    }

    cmd.forward_m_s = 0.0;
    offboard.set_velocity_body(cmd);
    std::this_thread::sleep_for(200ms);
    offboard.stop();

    reply -> set_accepted(true);
    reply -> set_message("Turning completed");
    return grpc::Status::OK;
}

grpc::Status DroneControlService::StopNow(grpc::ServerContext*,
                                         const drone::StopRequest*,
                                         drone::StopReply* reply)
{
    auto r = _action.hold();
    reply->set_ok(r == mavsdk::Action::Result::Success);
    reply->set_message(std::string("Hold result: ") + action_result_to_string(r));
    std::cout << "[API] StopNow -> hold()\n";
    return grpc::Status::OK;
}
