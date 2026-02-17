#include "flight_helpers.h"

#include <thread>

using namespace std::chrono_literals;

const char* action_result_to_string(mavsdk::Action::Result r)
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

bool ensure_airborne_or_reply(
    mavsdk::Action& action,
    mavsdk::Telemetry& telemetry,
    drone::CommandAck* reply)
{
    // Wait until PX4 is happy (SITL often needs time)
    const auto start = std::chrono::steady_clock::now();
    while (!telemetry.health_all_ok()) {
        if (std::chrono::steady_clock::now() - start > 10s) {
            reply->set_accepted(false);
            reply->set_message("Preflight not OK (health_all_ok=false) after 10s");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }

    // Arm if needed
    if (!telemetry.armed()) {
        auto r = action.arm();
        if (r != mavsdk::Action::Result::Success) {
            reply->set_accepted(false);
            reply->set_message(std::string("Arm failed: ") + action_result_to_string(r));
            return false;
        }

        // Wait until armed flag updates
        const auto arm_start = std::chrono::steady_clock::now();
        while (!telemetry.armed()) {
            if (std::chrono::steady_clock::now() - arm_start > 5s) {
                reply->set_accepted(false);
                reply->set_message("Arm sent, but telemetry.armed() stayed false");
                return false;
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    // Takeoff if needed
    if (!telemetry.in_air()) {
        auto r = action.takeoff();
        if (r != mavsdk::Action::Result::Success) {
            reply->set_accepted(false);
            reply->set_message(std::string("Takeoff failed: ") + action_result_to_string(r));
            return false;
        }

        // Wait until actually in air
        const auto to_start = std::chrono::steady_clock::now();
        while (!telemetry.in_air()) {
            if (std::chrono::steady_clock::now() - to_start > 10s) {
                reply->set_accepted(false);
                reply->set_message("Takeoff sent, but telemetry.in_air() stayed false");
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }
    }

    return true;
}

bool wait_until_ready_for_offboard_or_reply(
    mavsdk::Telemetry& telemetry,
    drone::CommandAck* reply)
{
    // Wait until Takeoff mode is finished (PX4 often stays here briefly)
    const auto start = std::chrono::steady_clock::now();
    while (telemetry.flight_mode() == mavsdk::Telemetry::FlightMode::Takeoff) {
        if (std::chrono::steady_clock::now() - start > 10s) {
            reply->set_accepted(false);
            reply->set_message("Still in Takeoff flight mode after 10s");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }
    return true;
}
