#include "flight_controller.h"
#include "flight_helpers.h"

#include <iostream>

using namespace std::chrono_literals;

FlightController::FlightController(std::shared_ptr<mavsdk::System> system)
    : _system(std::move(system)),
      _action(_system),
      _telemetry(_system),
      _offboard(_system)
{
    _action.set_takeoff_altitude(2.0f);

    _current_cmd.forward_m_s = 0.0f;
    _current_cmd.right_m_s = 0.0f;
    _current_cmd.down_m_s = 0.0f;
    _current_cmd.yawspeed_deg_s = 0.0f;
}

FlightController::~FlightController()
{
    stop_offboard_best_effort();
    stop_streaming_thread();
}

std::string FlightController::last_error() const
{
    std::lock_guard<std::mutex> lk(_err_mutex);
    return _last_error;
}

void FlightController::set_error(std::string msg)
{
    std::lock_guard<std::mutex> lk(_err_mutex);
    _last_error = std::move(msg);
}

void FlightController::set_cmd(float forward, float right, float down, float yawspeed_deg_s)
{
    std::lock_guard<std::mutex> lk(_cmd_mutex);
    _current_cmd.forward_m_s = forward;
    _current_cmd.right_m_s = right;
    _current_cmd.down_m_s = down;
    _current_cmd.yawspeed_deg_s = yawspeed_deg_s;
}

mavsdk::Offboard::VelocityBodyYawspeed FlightController::get_cmd_copy()
{
    std::lock_guard<std::mutex> lk(_cmd_mutex);
    return _current_cmd;
}

void FlightController::start_streaming_thread()
{
    if (_stream_running.exchange(true)) return;

    _stream_thread = std::thread([this]() {
        while (_stream_running.load()) {
            if (_offboard_active.load()) {
                auto cmd = get_cmd_copy();
                _offboard.set_velocity_body(cmd);
            }
            std::this_thread::sleep_for(50ms);
        }
    });
}

void FlightController::stop_streaming_thread()
{
    if (!_stream_running.exchange(false)) return;
    if (_stream_thread.joinable()) _stream_thread.join();
}

bool FlightController::ensure_armed(std::chrono::seconds timeout)
{
    if (_telemetry.armed()) return true;

    const auto start = std::chrono::steady_clock::now();
    while (!_telemetry.health_all_ok()) {
        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Preflight not OK (health_all_ok=false)");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }

    auto r = _action.arm();
    if (r != mavsdk::Action::Result::Success) {
        set_error(std::string("Arm failed: ") + action_result_to_string(r));
        return false;
    }

    const auto arm_start = std::chrono::steady_clock::now();
    while (!_telemetry.armed()) {
        if (std::chrono::steady_clock::now() - arm_start > 5s) {
            set_error("Arm sent, but telemetry.armed() stayed false");
            return false;
        }
        std::this_thread::sleep_for(100ms);
    }
    return true;
}

bool FlightController::ensure_airborne(std::chrono::seconds timeout)
{
    if (_telemetry.in_air()) return true;

    auto r = _action.takeoff();
    if (r != mavsdk::Action::Result::Success) {
        set_error(std::string("Takeoff failed: ") + action_result_to_string(r));
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    while (!_telemetry.in_air()) {
        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Takeoff sent, but telemetry.in_air() stayed false");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }
    return true;
}

bool FlightController::wait_until_ready_for_offboard(std::chrono::seconds timeout)
{

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        if (_telemetry.in_air() && _telemetry.health_all_ok()) return true;

        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Not ready for offboard within timeout");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }
}

bool FlightController::start_offboard_session(std::chrono::seconds timeout)
{
    if (_offboard_active.load()) return true;

    start_streaming_thread();

    set_cmd(0, 0, 0, 0);
    for (int i = 0; i < 20; ++i) {
        _offboard.set_velocity_body(get_cmd_copy());
        std::this_thread::sleep_for(50ms);
    }

    auto r = _offboard.start();
    if (r != mavsdk::Offboard::Result::Success) {
        set_error(std::string("Offboard start failed: ") + offboard_result_to_string(r));
        return false;
    }

    // wait a bit to see if mode switches (optional but useful)
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const auto mode = _telemetry.flight_mode();
        if (mode == mavsdk::Telemetry::FlightMode::Offboard) break;

        if (std::chrono::steady_clock::now() - start > timeout) {
            // We can still keep streaming, but this usually means PX4 rejected/left offboard
            set_error("Offboard start returned Success, but flight mode did not become Offboard");
            // treat as failure to be strict
            return false;
        }
        std::this_thread::sleep_for(100ms);
    }

    _offboard_active.store(true);
    return true;
}

void FlightController::stop_offboard_best_effort()
{
    if (!_offboard_active.exchange(false)) return;

    set_cmd(0, 0, 0, 0);
    std::this_thread::sleep_for(200ms);
    _offboard.stop();
}

void FlightController::schedule_stop_after(std::chrono::duration<double> d,
                                           bool stop_yaw_only,
                                           bool stop_forward_only)
{
    std::thread([this, d, stop_yaw_only, stop_forward_only]() {
        std::this_thread::sleep_for(d);

        auto cmd = get_cmd_copy();
        if (stop_yaw_only) {
            cmd.yawspeed_deg_s = 0.0f;
        } else if (stop_forward_only) {
            cmd.forward_m_s = 0.0f;
        } else {
            cmd.forward_m_s = 0.0f;
            cmd.right_m_s = 0.0f;
            cmd.down_m_s = 0.0f;
            cmd.yawspeed_deg_s = 0.0f;
        }

        {
            std::lock_guard<std::mutex> lk(_cmd_mutex);
            _current_cmd = cmd;
        }
    }).detach();
}

bool FlightController::request_hover(double seconds)
{
    if (seconds <= 0.0) {
        set_error("Hover seconds must be > 0");
        return false;
    }

    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;
    while (_telemetry.flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
        std::this_thread::sleep_for(200ms);
    }

    set_cmd(0, 0, 0, 0);
    schedule_stop_after(std::chrono::duration<double>(seconds)); // just keeps zero anyway
    return true;
}

bool FlightController::request_fly_forward(double seconds, double velocity_m_s)
{
    if (seconds <= 0.0) {
        set_error("Fly seconds must be > 0");
        return false;
    }
    if (velocity_m_s == 0.0) {
        set_error("Velocity must be non-zero");
        return false;
    }
    std::cout << "[API] Command accepted (queued)\n";
    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;
    while (_telemetry.flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
        std::cout << "The drone is being checked for flight satus and waits for Offboard";
        std::this_thread::sleep_for(200ms);
    }
    std::cout << "[API] Command accepted (queued) after checks\n";

    set_cmd(static_cast<float>(velocity_m_s), 0.0f, 0.0f, 0.0f);
    schedule_stop_after(std::chrono::duration<double>(seconds), false, true); // stop forward
    return true;
}

bool FlightController::request_turn_yaw(double degrees, double seconds)
{
    if (seconds <= 0.0) {
        set_error("Turn seconds must be > 0");
        return false;
    }

    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;

    const float yaw_rate = static_cast<float>(degrees / seconds);
    set_cmd(0.0f, 0.0f, 0.0f, yaw_rate);
    schedule_stop_after(std::chrono::duration<double>(seconds), true, false); // stop yaw
    return true;
}

bool FlightController::request_stop()
{
    set_cmd(0, 0, 0, 0);
    return true;
}

bool FlightController::request_land()
{
    set_cmd(0, 0, 0, 0);
    stop_offboard_best_effort();

    auto r = _action.land();
    if (r != mavsdk::Action::Result::Success) {
        set_error(std::string("Land failed: ") + action_result_to_string(r));
        return false;
    }
    return true;
}