#include "flight_controller.h"
#include "flight_helpers.h"

#include <iostream>
#include <vector>

using namespace std::chrono_literals;

namespace {
constexpr float kTakeoffAltitudeM = 2.0f;
constexpr float kTakeoffToleranceM = 0.2f;
constexpr auto kStreamPeriod = 50ms;
}

FlightController::FlightController(std::shared_ptr<mavsdk::System> system)
    : _system(std::move(system)),
      _action(_system),
      _telemetry(_system),
      _offboard(_system)
{
    _action.set_takeoff_altitude(kTakeoffAltitudeM);

    _current_cmd.forward_m_s = 0.0f;
    _current_cmd.right_m_s = 0.0f;
    _current_cmd.down_m_s = 0.0f;
    _current_cmd.yawspeed_deg_s = 0.0f;

    if (_system && _system->has_autopilot()) {
        _state.store(ExecState::Idle);
    } else {
        _state.store(ExecState::Disconnected);
    }

    start_worker();
}

FlightController::~FlightController()
{
    stop_worker();
    stop_offboard_session_best_effort();
    stop_streaming_thread();
}

void FlightController::set_state(ExecState new_state)
{
    _state.store(new_state);
}

void FlightController::set_error(std::string msg)
{
    {
        std::lock_guard<std::mutex> lock(_err_mutex);
        _last_error = std::move(msg);
    }
    _state.store(ExecState::Error);
}

void FlightController::clear_error()
{
    std::lock_guard<std::mutex> lock(_err_mutex);
    _last_error.clear();
}

std::string FlightController::last_error() const
{
    std::lock_guard<std::mutex> lock(_err_mutex);
    return _last_error;
}

void FlightController::set_command_status(const std::string& id,
                                          CommandState state,
                                          std::string message,
                                          std::uint32_t attempt_count,
                                          std::uint32_t max_attempts)
{
    std::lock_guard<std::mutex> lock(_command_status_mutex);
    _command_status[id] = CommandStatusSnapshot{
        true,
        state,
        std::move(message),
        attempt_count,
        max_attempts
    };
}

FlightController::CommandStatusSnapshot
FlightController::get_command_status(const std::string& id) const
{
    std::lock_guard<std::mutex> lock(_command_status_mutex);
    auto it = _command_status.find(id);
    if (it == _command_status.end()) {
        return {};
    }
    return it->second;
}

FlightController::ExecState FlightController::current_state() const
{
    return _state.load();
}

FlightController::StatusSnapshot FlightController::get_status() const
{
    StatusSnapshot status{};
    status.state = _state.load();
    status.last_error = last_error();
    status.connected = (_system && _system->has_autopilot());
    status.armed = _telemetry.armed();
    status.in_air = _telemetry.in_air();
    status.relative_altitude_m = _telemetry.position().relative_altitude_m;
    status.flight_mode = static_cast<int>(_telemetry.flight_mode());

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        status.queue_size = static_cast<std::uint32_t>(_command_queue.size());
    }

    return status;
}

void FlightController::set_cmd(float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s)
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    _current_cmd.forward_m_s = forward_m_s;
    _current_cmd.right_m_s = right_m_s;
    _current_cmd.down_m_s = down_m_s;
    _current_cmd.yawspeed_deg_s = yawspeed_deg_s;
}

mavsdk::Offboard::VelocityBodyYawspeed FlightController::get_cmd_copy() const
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    return _current_cmd;
}

void FlightController::start_streaming_thread()
{
    if (_stream_thread.joinable()) {
        return;
    }

    _stop_stream_thread.store(false);
    _stream_thread = std::thread(&FlightController::offboard_stream_loop, this);
}

void FlightController::stop_streaming_thread()
{
    _stop_stream_thread.store(true);

    if (_stream_thread.joinable()) {
        _stream_thread.join();
    }
}

void FlightController::offboard_stream_loop()
{
    while (!_stop_stream_thread.load()) {
        const auto cmd = get_cmd_copy();
        _offboard.set_velocity_body(cmd);
        std::this_thread::sleep_for(kStreamPeriod);
    }
}

bool FlightController::ensure_armed(std::chrono::seconds timeout)
{
    if (_telemetry.armed()) {
        return true;
    }

    set_state(ExecState::Arming);

    const auto start = std::chrono::steady_clock::now();
    while (!_telemetry.health_all_ok()) {
        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Preflight not OK (health_all_ok=false)");
            return false;
        }
        std::this_thread::sleep_for(200ms);
    }

    const auto result = _action.arm();
    if (result != mavsdk::Action::Result::Success) {
        set_error(std::string("Arm failed: ") + action_result_to_string(result));
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
    if (_telemetry.in_air() &&
        _telemetry.position().relative_altitude_m >= (kTakeoffAltitudeM - kTakeoffToleranceM)) {
        return true;
    }

    set_state(ExecState::TakingOff);

    const auto result = _action.takeoff();
    if (result != mavsdk::Action::Result::Success) {
        set_error(std::string("Takeoff failed: ") + action_result_to_string(result));
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    while (_telemetry.position().relative_altitude_m < (kTakeoffAltitudeM - kTakeoffToleranceM)) {
        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Takeoff sent, but target altitude was never reached");
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
        if (_telemetry.in_air() && _telemetry.health_all_ok()) {
            return true;
        }

        if (std::chrono::steady_clock::now() - start > timeout) {
            set_error("Not ready for offboard within timeout");
            return false;
        }

        std::this_thread::sleep_for(200ms);
    }
}

bool FlightController::start_offboard_session(std::chrono::seconds timeout)
{
    if (_offboard_running.load()) {
        return true;
    }

    if (!wait_until_ready_for_offboard(timeout)) {
        return false;
    }

    for (int i = 0; i < 20; ++i) {
        _offboard.set_velocity_body(get_cmd_copy());
        std::this_thread::sleep_for(kStreamPeriod);
    }

    const auto result = _offboard.start();
    if (result != mavsdk::Offboard::Result::Success) {
        set_error(std::string("Offboard start failed: ") + offboard_result_to_string(result));
        return false;
    }

    _offboard_running.store(true);
    start_streaming_thread();

    const auto start = std::chrono::steady_clock::now();
    while (_telemetry.flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
        if (std::chrono::steady_clock::now() - start > 5s) {
            set_error("Timed out waiting for Offboard mode");
            return false;
        }
        std::this_thread::sleep_for(100ms);
    }

    return true;
}

void FlightController::stop_offboard_session_best_effort()
{
    if (!_offboard_running.load()) {
        return;
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    stop_streaming_thread();
    (void)_offboard.stop();
    _offboard_running.store(false);
}

bool FlightController::enqueue_hover(const std::string& id, double seconds)
{
    if (seconds <= 0.0) {
        set_error("Hover seconds must be > 0");
        set_command_status(id, CommandState::Failed, last_error(),0,0);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({id, QueuedCommand::Type::Hover, seconds, 0.0, 0,3});
    }
    set_command_status(id, CommandState::Queued, "Queued",0,0);
    _queue_cv.notify_one();
    return true;
}

bool FlightController::enqueue_fly_forward(const std::string& id, double seconds, double velocity_m_s)
{
    if (seconds <= 0.0) {
        set_error("Fly seconds must be > 0");
        set_command_status(id, CommandState::Failed, last_error(),0,0);
        return false;
    }
    if (velocity_m_s == 0.0) {
        set_error("Velocity must be non-zero");
        set_command_status(id, CommandState::Failed, last_error(),0,0);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({id, QueuedCommand::Type::FlyForward, seconds, velocity_m_s,0,3});
    }
    set_command_status(id, CommandState::Queued, "Queued",0,0);
    _queue_cv.notify_one();
    return true;
}

bool FlightController::enqueue_turn_yaw(const std::string& id, double degrees, double seconds)
{
    if (seconds <= 0.0) {
        set_error("Turn seconds must be > 0");
        set_command_status(id, CommandState::Failed, last_error(),0,0);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({id, QueuedCommand::Type::TurnYaw, degrees, seconds,0,3});
    }
    set_command_status(id, CommandState::Queued, "Queued",0,0);
    _queue_cv.notify_one();
    return true;
}

bool FlightController::enqueue_land(const std::string& id)
{
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _command_queue.push_back({id, QueuedCommand::Type::Land, 0.0, 0.0,0,3});
    }
    set_command_status(id, CommandState::Queued, "Queued",0,0);
    _queue_cv.notify_one();
    return true;
}
FlightController::ExecResult
FlightController::execute_hover(double seconds)
{
    if (!check_health(10s,10s,10s,2s)) {
        return {false,true,last_error()};
    }

    set_state(ExecState::Hovering);
    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);

    FlightController::ExecResult result;
    if (check_if_interrupted(seconds, result)) return result;

    return {true,false,"Succeeded"};
}

FlightController::ExecResult
FlightController::execute_fly_forward(double seconds, double velocity_m_s)
{
    if (!check_health(10s,10s,10s,2s)) {
        return {false,true,last_error()};
    }

    set_state(ExecState::ExecutingCommand);
    set_cmd(static_cast<float>(velocity_m_s), 0.0f, 0.0f, 0.0f);

    FlightController::ExecResult result;
    if (check_if_interrupted(seconds, result)) return result;

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    set_state(ExecState::Hovering);
    return {false,true,last_error()};
}
FlightController::ExecResult
FlightController::execute_turn_yaw(double degrees, double seconds)
{

    if (!check_health(10s,10s,10s,2s)) {
        return {false,true,last_error()};
    }

    set_state(ExecState::ExecutingCommand);
    const float yaw_rate = static_cast<float>(degrees / seconds);
    set_cmd(0.0f, 0.0f, 0.0f, yaw_rate);

    FlightController::ExecResult result;
    if (check_if_interrupted(seconds, result)) return result;

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    set_state(ExecState::Hovering);
    return {true,false,"Succeeded"};
}
FlightController::ExecResult
FlightController::execute_land()
{
    set_state(ExecState::Landing);
    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    stop_offboard_session_best_effort();

    const auto result = _action.land();
    if (result != mavsdk::Action::Result::Success) {
        set_error(std::string("Land failed: ") + action_result_to_string(result));
        return {false, true, last_error()};
    }

    const auto start = std::chrono::steady_clock::now();
    while (_telemetry.in_air()) {
        if (_interrupt_current.load()) {
            return {false,false, "Interrupted"};
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            set_error("Landing timeout: telemetry.in_air() stayed true");
            return {false, true,"Stayed in air"};
        }

        std::this_thread::sleep_for(200ms);
    }

    set_state(ExecState::Idle);
    return {true, false, "Succeeded"};
}

bool FlightController::request_stop()
{
    set_state(ExecState::EmergencyStopped);
    _interrupt_current.store(true);

    std::vector<std::string> cancelled_ids;
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        for (const auto& cmd : _command_queue) {
            cancelled_ids.push_back(cmd.id);
        }
        _command_queue.clear();
    }

    for (const auto& id : cancelled_ids) {
        set_command_status(id, CommandState::Cancelled, "Cancelled by StopNow",0,0);
    }

    set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    stop_offboard_session_best_effort();
    return true;
}

void FlightController::start_worker()
{
    if (_worker_running.exchange(true)) {
        return;
    }

    _worker_thread = std::thread(&FlightController::worker_loop, this);
}

void FlightController::stop_worker()
{
    if (!_worker_running.exchange(false)) {
        return;
    }

    _queue_cv.notify_all();

    if (_worker_thread.joinable()) {
        _worker_thread.join();
    }
}

void FlightController::worker_loop()
{
    if (_system && _system->has_autopilot()) {
        set_state(ExecState::Idle);
    } else {
        set_state(ExecState::Disconnected);
    }

    while (_worker_running.load()) {
        QueuedCommand cmd{};

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

        clear_error();
        _interrupt_current.store(false);
        set_command_status(cmd.id, CommandState::Running, "Running",
            cmd.attempt_count,cmd.max_attempts);

        while (cmd.attempt_count < cmd.max_attempts) {
            clear_error();
            _interrupt_current.store(false);

            const ExecResult result = execute_command(cmd);

            if (result.ok) {
                set_command_status(cmd.id, CommandState::Succeeded,
                    result.message,cmd.attempt_count,cmd.max_attempts);

                if (_state.load() != ExecState::EmergencyStopped) {
                    set_state(_telemetry.in_air() ? ExecState::Hovering : ExecState::Idle);
                }
                break;
            }

            if (_interrupt_current.load()) {
                set_command_status(cmd.id, CommandState::Interrupted, "Interrupted",
                    cmd.attempt_count,cmd.max_attempts);
                break;
            }

            cmd.attempt_count++;

            if (!result.retryable || cmd.attempt_count >= cmd.max_attempts) {
                set_command_status(cmd.id, CommandState::Failed, result.message);

                if (_state.load() != ExecState::EmergencyStopped) {
                    set_state(ExecState::Error);
                }
                break;
            }

            set_command_status(
                cmd.id,
                CommandState::Retrying,
                "Request failed, trying again (" +
                    std::to_string(cmd.attempt_count + 1) + "/" +
                    std::to_string(cmd.max_attempts) + "): " +
                    result.message,cmd.attempt_count,cmd.max_attempts
            );

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            set_command_status(cmd.id, CommandState::Running, "Running",cmd.attempt_count,cmd.max_attempts);
        }
    }
}

FlightController::ExecResult
FlightController::execute_command(const QueuedCommand& cmd) {
    switch (cmd.type) {
        case QueuedCommand::Type::Hover:
            return execute_hover(cmd.a);
        case QueuedCommand::Type::Land:
            return execute_land();
        case QueuedCommand::Type::FlyForward:
            return execute_fly_forward(cmd.a,cmd.b);
        case QueuedCommand::Type::TurnYaw:
            return execute_fly_forward(cmd.a,cmd.b);
    }
    return{false,false,"Unknown command type"};
}

bool FlightController::check_health(std::chrono::seconds ensure_armed_s, std::chrono::seconds ensure_airborne_s
    , std::chrono::seconds wait_offboard_s, std::chrono::seconds start_offboard_s) {
    if (!ensure_armed(ensure_armed_s)) return false;
    if (!ensure_airborne(ensure_airborne_s)) return false;
    if (!wait_until_ready_for_offboard(wait_offboard_s)) return false;
    if (!start_offboard_session(start_offboard_s)) return false;
    return true;
}

bool FlightController::check_if_interrupted(double seconds, FlightController::ExecResult &result) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        if (_interrupt_current.load()) {
            set_cmd(0.0f, 0.0f, 0.0f, 0.0f);
            result = {false,false,"Interrupted"};
            return true;
        }
        std::this_thread::sleep_for(kStreamPeriod);
    }
    return false;
}

