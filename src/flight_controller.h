#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <mavsdk/system.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

class FlightController {
public:
    explicit FlightController(std::shared_ptr<mavsdk::System> system);
    ~FlightController();

    bool enqueue_hover(double seconds);
    bool enqueue_fly_forward(double seconds, double velocity_m_s);
    bool enqueue_turn_yaw(double degrees, double seconds);
    bool enqueue_land();

    bool request_stop();

    std::string last_error() const;

private:
    struct QueuedCommand {
        enum class Type {
            Hover,
            FlyForward,
            TurnYaw,
            Land
        };

        Type type;
        double a{0.0};
        double b{0.0};
    };

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

    void offboard_stream_loop();
    void start_streaming_thread();
    void stop_streaming_thread();

    void set_cmd(float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s);
    mavsdk::Offboard::VelocityBodyYawspeed get_cmd_copy();

    void set_error(std::string msg);

private:
    std::shared_ptr<mavsdk::System> _system;
    mavsdk::Action _action;
    mavsdk::Telemetry _telemetry;
    mavsdk::Offboard _offboard;

    std::mutex _cmd_mutex{};
    mavsdk::Offboard::VelocityBodyYawspeed _current_cmd{};

    std::atomic<bool> _offboard_running{false};
    std::atomic<bool> _stop_stream_thread{false};
    std::thread _stream_thread;

    std::deque<QueuedCommand> _command_queue;
    std::mutex _queue_mutex;
    std::condition_variable _queue_cv;
    std::atomic<bool> _worker_running{false};
    std::thread _worker_thread;

    std::atomic<bool> _interrupt_current{false};

    mutable std::mutex _err_mutex;
    std::string _last_error;
};