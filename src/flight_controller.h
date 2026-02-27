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

    std::string last_error() const;

private:
    std::shared_ptr<mavsdk::System> _system;
    mavsdk::Action _action;
    mavsdk::Telemetry _telemetry;
    mavsdk::Offboard _offboard;

    mutable std::mutex _cmd_mutex;
    mavsdk::Offboard::VelocityBodyYawspeed _current_cmd{};

    std::atomic<bool> _stream_running{false};
    std::thread _stream_thread;

    std::atomic<bool> _offboard_active{false};

    mutable std::mutex _err_mutex;
    std::string _last_error;

    bool ensure_armed(std::chrono::seconds timeout);
    bool ensure_airborne(std::chrono::seconds timeout);
    bool start_offboard_session(std::chrono::seconds timeout);
    void stop_offboard_best_effort();

    bool wait_until_ready_for_offboard(std::chrono::seconds timeout);

    void start_streaming_thread();
    void stop_streaming_thread();

    void set_error(std::string msg);
    void set_cmd(float forward, float right, float down, float yawspeed_deg_s);
    mavsdk::Offboard::VelocityBodyYawspeed get_cmd_copy();

    void schedule_stop_after(std::chrono::duration<double> d,
                            bool stop_yaw_only = false,
                            bool stop_forward_only = false);
};