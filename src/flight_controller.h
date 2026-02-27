#pragma once

#include <atomic>
#include <chrono>
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

    bool request_hover(double seconds);
    bool request_fly_forward(double seconds, double velocity_m_s);
    bool request_turn_yaw(double degrees, double seconds);
    bool request_land();
    bool request_stop();
    void stop_offboard_session_best_effort();
    bool start_offboard_session(std::chrono::seconds timeout);

    void set_cmd(float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s);

    bool ensure_armed(std::chrono::seconds timeout);
    bool ensure_airborne(std::chrono::seconds timeout);
    bool wait_until_ready_for_offboard(std::chrono::seconds timeout);
    bool offboard_running() const { return _offboard_running.load(); }

    std::string last_error() const;

private:
    void offboard_stream_loop();
    std::shared_ptr<mavsdk::System> _system;
    mavsdk::Action _action;
    mavsdk::Telemetry _telemetry;
    mavsdk::Offboard _offboard;

    std::mutex _cmd_mutex{};
    mavsdk::Offboard::VelocityBodyYawspeed _current_cmd{};

    std::atomic<bool> _offboard_running{false};
    std::atomic<bool> _stop_thread{false};
    std::atomic<bool> _stream_running{false};
    std::thread _stream_thread;

    std::atomic<bool> _offboard_active{false};

    mutable std::mutex _err_mutex;
    std::string _last_error;

    void stop_offboard_best_effort();


    void start_streaming_thread();
    void stop_streaming_thread();

    void set_error(std::string msg);
    mavsdk::Offboard::VelocityBodyYawspeed get_cmd_copy();

    void schedule_stop_after(std::chrono::duration<double> d,
                            bool stop_yaw_only = false,
                            bool stop_forward_only = false);
};