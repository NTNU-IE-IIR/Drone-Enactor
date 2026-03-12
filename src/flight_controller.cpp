#include "flight_controller.h"
#include "flight_helpers.h"

#include <iostream>

using namespace std::chrono_literals;
double takeoff_altitude = 2.0f;

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

    start_worker();
}

FlightController::~FlightController()
{
    stop_worker();
    stop_offboard_session_best_effort();
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

void FlightController::set_cmd(float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s)
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    _current_cmd.forward_m_s = forward_m_s;
    _current_cmd.right_m_s = right_m_s;
    _current_cmd.down_m_s = down_m_s;
    _current_cmd.yawspeed_deg_s = yawspeed_deg_s;
}

mavsdk::Offboard::VelocityBodyYawspeed FlightController::get_cmd_copy()
{
    std::lock_guard<std::mutex> lk(_cmd_mutex);
    return _current_cmd;
}

void FlightController::start_streaming_thread()
{
    if (_stream_thread.joinable()) return;

    _stop_stream_thread.store(false);

    _stream_thread = std::thread([this]() {
        while (!_stop_stream_thread.load()) {
            auto cmd = get_cmd_copy();
            _offboard.set_velocity_body(cmd);
            std::this_thread::sleep_for(50ms);
        }
    });
}

void FlightController::stop_streaming_thread()
{
    _stop_stream_thread.store(true);

    if (_stream_thread.joinable()) {
        _stream_thread.join();
    }
}

bool FlightController::ensure_armed(std::chrono::seconds timeout)
{
    if (_telemetry.armed()) {
        std::cout << "The arming was succesfully completed \n";
        return true;
    }

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
    std::cout << "The arming was succesfully completed \n";
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

    const float target = static_cast<float>(takeoff_altitude);
    const float tol = 0.2f;

    const auto start = std::chrono::steady_clock::now();
    while (_telemetry.position().relative_altitude_m < target - tol ) {
        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Takeoff sent, but telemetry.in_air() stayed false");
            return false;
        }
        std::cout << "The vechicle is not in air\n";
        std::this_thread::sleep_for(1000ms);
    }
    std::cout << "The vechicle is in air 2\n";
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
void FlightController::offboard_stream_loop()
{
    while (!_stop_stream_thread.load()) {
        auto cmd_copy = get_cmd_copy();
        _offboard.set_velocity_body(cmd_copy);
        std::this_thread::sleep_for(50ms);
    }
}
void FlightController::stop_offboard_session_best_effort()
{
    if (!_offboard_running.load()) {
        return;
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);

    stop_streaming_thread();
    _offboard.stop();

    _offboard_running.store(false);
    std::cout << "[FC] Offboard session stopped\n";
}
bool FlightController::start_offboard_session(std::chrono::seconds timeout)
{
    if (_offboard_running.load()) {
        return true;
    }

    if (!wait_until_ready_for_offboard(timeout)) {
        std::cout << "[FC] Not ready for offboard\n";
        return false;
    }

    for (int i = 0; i < 20; ++i) {
        mavsdk::Offboard::VelocityBodyYawspeed cmd_copy{};
        {
            std::lock_guard<std::mutex> lock(_cmd_mutex);
            cmd_copy = _current_cmd;
        }
        _offboard.set_velocity_body(cmd_copy);
        std::this_thread::sleep_for(50ms);
    }

    auto r = _offboard.start();
    if (r != mavsdk::Offboard::Result::Success) {
        std::cout << "[FC] Offboard.start() failed\n";
        return false;
    }
    _stop_stream_thread.store(false);
    _offboard_running.store(true);
    start_streaming_thread();
    while (_telemetry.flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
        std::cout << "The drone is being checked for flight satus and waits for Offboard\n";
        std::this_thread::sleep_for(600ms);
    }
    std::cout << "[FC] Offboard session started (streaming 20Hz)\n";
    return true;
}


bool FlightController::enqueue_hover(double seconds)
{
    if (seconds <= 0.0) {
        set_error("Hover seconds must be > 0");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({QueuedCommand::Type::Hover, seconds, 0.0});
    }
    _queue_cv.notify_one();
    return true;
}
bool FlightController::execute_land()
{
    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    stop_offboard_session_best_effort();

    auto r = _action.land();
    if (r != mavsdk::Action::Result::Success) {
        set_error(std::string("Land failed: ") + action_result_to_string(r));
        return false;
    }

    return true;
}

bool FlightController::enqueue_fly_forward(double seconds, double velocity_m_s)
{
    if (seconds <= 0.0) {
        set_error("Fly seconds must be > 0");
        return false;
    }
    if (velocity_m_s == 0.0) {
        set_error("Velocity must be non-zero");
        return false;
    }
    std::cout << "You are now oin the enqueue_fly_forward \n";

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({QueuedCommand::Type::FlyForward, seconds, velocity_m_s});
    }
    _queue_cv.notify_one();
    return true;
}

bool FlightController::enqueue_turn_yaw(double degrees, double seconds)
{
    if (seconds <= 0.0) {
        set_error("Turn seconds must be > 0");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({QueuedCommand::Type::TurnYaw, degrees, seconds});
    }
    _queue_cv.notify_one();
    return true;
}

bool FlightController::enqueue_land()
{
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({QueuedCommand::Type::Land, 0.0, 0.0});
    }
    _queue_cv.notify_one();
    return true;
}
bool FlightController::request_stop()
{
    _interrupt_current.store(true);

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.clear();
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    return true;
}
void FlightController::start_worker()
{
    if (_worker_running.exchange(true)) return;

    _worker_thread = std::thread(&FlightController::worker_loop, this);
}

void FlightController::stop_worker()
{
    if (!_worker_running.exchange(false)) return;

    _queue_cv.notify_all();
    if (_worker_thread.joinable()) {
        _worker_thread.join();
    }
}

void FlightController::worker_loop()
{
    while (_worker_running.load()) {
        QueuedCommand cmd;

        {
            std::unique_lock<std::mutex> lock(_queue_mutex);

            _queue_cv.wait(lock, [this] {
                return !_command_queue.empty() || !_worker_running.load();
            });

            if (!_worker_running.load()) {
                return;
            }

            cmd = _command_queue.front();
            _command_queue.pop_front();
        }

        _interrupt_current.store(false);

        switch (cmd.type) {
            case QueuedCommand::Type::Hover:
                execute_hover(cmd.a);
            break;
            case QueuedCommand::Type::FlyForward:
                execute_fly_forward(cmd.a, cmd.b);
            break;
            case QueuedCommand::Type::TurnYaw:
                execute_turn_yaw(cmd.a, cmd.b);
            break;
            case QueuedCommand::Type::Land:
                execute_land();
            break;
        }
    }
}

bool FlightController::execute_hover(double seconds)
{
    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);

    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        if (_interrupt_current.load()) {
            set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
            return false;
        }
        std::this_thread::sleep_for(50ms);
    }

    return true;
}
bool FlightController::execute_fly_forward(double seconds, double velocity_m_s)
{
    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;

    set_cmd(static_cast<float>(velocity_m_s), 0.0f, 0.0f, 0.0f);

    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        if (_interrupt_current.load()) {
            set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
            return false;
        }
        std::this_thread::sleep_for(50ms);
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    return true;
}
bool FlightController::execute_turn_yaw(double degrees, double seconds)
{
    if (!ensure_armed(10s)) return false;
    if (!ensure_airborne(10s)) return false;
    if (!wait_until_ready_for_offboard(10s)) return false;
    if (!start_offboard_session(2s)) return false;

    const float yaw_rate = static_cast<float>(degrees / seconds);
    set_cmd(0.0f, 0.0f, 0.0f, yaw_rate);

    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        if (_interrupt_current.load()) {
            set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
            return false;
        }
        std::this_thread::sleep_for(50ms);
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    return true;
}
