#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/system.h>

class FlightController {
public:
    enum class ExecState {
        Disconnected,
        Idle,
        Arming,
        TakingOff,
        Hovering,
        ExecutingCommand,
        Landing,
        EmergencyStopped,
        Error
    };

    enum class CommandState {
        Unknown,
        Queued,
        Running,
        Succeeded,
        Failed,
        Interrupted,
        Cancelled
    };

    struct StatusSnapshot {
        ExecState state{ExecState::Disconnected};
        std::string last_error{};
        bool connected{false};
        bool armed{false};
        bool in_air{false};
        float relative_altitude_m{0.0f};
        int flight_mode{0};
        std::uint32_t queue_size{0};
    };

    struct CommandStatusSnapshot {
        bool found{false};
        CommandState state{CommandState::Unknown};
        std::string message{};
    };

    explicit FlightController(std::shared_ptr<mavsdk::System> system);
    ~FlightController();

    bool enqueue_hover(const std::string& id, double seconds);
    bool enqueue_fly_forward(const std::string& id, double seconds, double velocity_m_s);
    bool enqueue_turn_yaw(const std::string& id, double degrees, double seconds);
    bool enqueue_land(const std::string& id);

    bool request_stop();

    std::string last_error() const;
    StatusSnapshot get_status() const;
    CommandStatusSnapshot get_command_status(const std::string& id) const;
    ExecState current_state() const;

private:
    struct QueuedCommand {
        enum class Type {
            Hover,
            FlyForward,
            TurnYaw,
            Land
        };

        std::string id;
        Type type;
        double a{0.0};
        double b{0.0};
    };

    void set_state(ExecState new_state);
    void set_error(std::string msg);
    void clear_error();

    void set_command_status(const std::string& id, CommandState state, std::string message);

    void worker_loop();
    void start_worker();
    void stop_worker();

    bool execute_hover(double seconds);
    bool execute_fly_forward(double seconds, double velocity_m_s);
    bool execute_turn_yaw(double degrees, double seconds);
    bool execute_land();

    bool ensure_armed(std::chrono::seconds timeout);
    bool ensure_airborne(std::chrono::seconds timeout);
    bool wait_until_ready_for_offboard(std::chrono::seconds timeout);
    bool start_offboard_session(std::chrono::seconds timeout);
    void stop_offboard_session_best_effort();

    void start_streaming_thread();
    void stop_streaming_thread();
    void offboard_stream_loop();

    void set_cmd(float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s);
    mavsdk::Offboard::VelocityBodyYawspeed get_cmd_copy() const;

private:
    std::shared_ptr<mavsdk::System> _system;
    mavsdk::Action _action;
    mavsdk::Telemetry _telemetry;
    mavsdk::Offboard _offboard;

    mutable std::mutex _cmd_mutex{};
    mavsdk::Offboard::VelocityBodyYawspeed _current_cmd{};

    std::atomic<bool> _offboard_running{false};
    std::atomic<bool> _stop_stream_thread{false};
    std::thread _stream_thread;

    std::deque<QueuedCommand> _command_queue;
    mutable std::mutex _queue_mutex;
    std::condition_variable _queue_cv;
    std::atomic<bool> _worker_running{false};
    std::thread _worker_thread;

    std::atomic<bool> _interrupt_current{false};

    std::atomic<ExecState> _state{ExecState::Disconnected};

    mutable std::mutex _err_mutex;
    std::string _last_error;

    mutable std::mutex _command_status_mutex;
    std::unordered_map<std::string, CommandStatusSnapshot> _command_status;
};