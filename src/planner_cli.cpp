#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <google/protobuf/empty.pb.h>
#include <grpcpp/grpcpp.h>

#include "drone_control.grpc.pb.h"

static std::mutex g_print_mutex;

static void print_help()
{
    std::cout
        << "Commands:\n"
        << "  hover <seconds>\n"
        << "  fly <seconds> <velocity_m_s>\n"
        << "  turn <degrees> <seconds>\n"
        << "  land\n"
        << "  stop\n"
        << "  status\n"
        << "  help\n"
        << "  quit\n";
}

static const char* exec_state_to_string(drone::ExecState state)
{
    switch (state) {
        case drone::DISCONNECTED: return "DISCONNECTED";
        case drone::IDLE: return "IDLE";
        case drone::ARMING: return "ARMING";
        case drone::TAKINGOFF: return "TAKINGOFF";
        case drone::HOVERING: return "HOVERING";
        case drone::EXECUTINGCOMMAND: return "EXECUTINGCOMMAND";
        case drone::LANDING: return "LANDING";
        case drone::EMERGENCYSTOPPED: return "EMERGENCYSTOPPED";
        case drone::ERROR: return "ERROR";
    }
    return "UNKNOWN_EXEC_STATE";
}

static const char* flight_mode_to_string(int mode)
{
    switch (mode) {
        case 0: return "UNKNOWN";
        case 1: return "READY";
        case 2: return "TAKEOFF";
        case 3: return "HOLD";
        case 4: return "MISSION";
        case 5: return "RETURN_TO_LAUNCH";
        case 6: return "LAND";
        case 7: return "OFFBOARD";
        case 8: return "FOLLOW_ME";
        case 9: return "MANUAL";
        case 10: return "ALTITUDE_CONTROL";
        case 11: return "POSITION_CONTROL";
        case 12: return "ACRO";
        case 13: return "STABILIZED";
        default: return "OTHER";
    }
}

static const char* command_state_to_string(drone::CommandState state)
{
    switch (state) {
        case drone::COMMAND_STATE_UNKNOWN: return "UNKNOWN";
        case drone::COMMAND_STATE_QUEUED: return "QUEUED";
        case drone::COMMAND_STATE_RUNNING: return "RUNNING";
        case drone::COMMAND_STATE_SUCCEEDED: return "SUCCEEDED";
        case drone::COMMAND_STATE_FAILED: return "FAILED";
        case drone::COMMAND_STATE_INTERRUPTED: return "INTERRUPTED";
        case drone::COMMAND_STATE_CANCELLED: return "CANCELLED";
    }
    return "UNKNOWN_COMMAND_STATE";
}

static bool is_terminal_command_state(drone::CommandState state)
{
    return state == drone::COMMAND_STATE_SUCCEEDED ||
           state == drone::COMMAND_STATE_FAILED ||
           state == drone::COMMAND_STATE_INTERRUPTED ||
           state == drone::COMMAND_STATE_CANCELLED;
}

static void safe_print(const std::string& text)
{
    std::lock_guard<std::mutex> lock(g_print_mutex);
    std::cout << text << std::flush;
}

static void watch_command(const std::string& addr, const std::string& id)
{
    auto channel = grpc::CreateChannel(addr, grpc::InsecureChannelCredentials());
    std::unique_ptr<drone::DroneControl::Stub> stub = drone::DroneControl::NewStub(channel);

    drone::CommandState last_state = drone::COMMAND_STATE_UNKNOWN;
    std::string last_message;

    for (;;) {
        drone::CommandStatusRequest req;
        req.set_id(id);

        drone::CommandStatusReply reply;
        grpc::ClientContext ctx;
        grpc::Status status = stub->GetCommandStatus(&ctx, req, &reply);

        if (!status.ok()) {
            safe_print(
                "GetCommandStatus grpc_ok=0 code=" +
                std::to_string(status.error_code()) +
                " msg=" + status.error_message() + "\n");
            return;
        }

        if (!reply.found()) {
            safe_print("Command " + id + " not found\n");
            return;
        }

        if (reply.command_state() != last_state || reply.message() != last_message) {
            safe_print(
                "Command " + id +
                " state=" + std::string(command_state_to_string(reply.command_state())) +
                " controller_state=" + std::string(exec_state_to_string(reply.controller_state())) +
                " msg=" + reply.message() + "\n");

            last_state = reply.command_state();
            last_message = reply.message();
        }

        if (is_terminal_command_state(reply.command_state())) {
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

int main(int argc, char** argv)
{
    std::vector<std::thread> watcher_threads;

    std::string addr = "127.0.0.1:60051";
    if (argc >= 2) {
        addr = argv[1];
    }

    auto channel = grpc::CreateChannel(addr, grpc::InsecureChannelCredentials());
    std::unique_ptr<drone::DroneControl::Stub> stub = drone::DroneControl::NewStub(channel);

    std::cout << "Planner CLI connected to " << addr << "\n";
    print_help();

    uint64_t next_id = 1;

    for (std::string line; std::cout << "> " && std::getline(std::cin, line);) {
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd.empty()) {
            continue;
        }

        if (cmd == "quit" || cmd == "exit") {
            break;
        }

        if (cmd == "help") {
            print_help();
            continue;
        }

        if (cmd == "stop") {
            drone::StopRequest req;
            drone::StopReply reply;

            grpc::ClientContext ctx;
            grpc::Status status = stub->StopNow(&ctx, req, &reply);

            std::cout << "StopNow grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " ok=" << reply.ok()
                      << " state=" << exec_state_to_string(reply.state())
                      << " reply_msg=" << reply.message()
                      << "\n";
            continue;
        }

        if (cmd == "status") {
            google::protobuf::Empty req;
            drone::StatusReply res;

            grpc::ClientContext ctx;
            grpc::Status status = stub->GetStatus(&ctx, req, &res);

            if (!status.ok()) {
                std::cout << "GetStatus grpc_ok=0"
                          << " code=" << status.error_code()
                          << " msg=" << status.error_message()
                          << "\n";
                continue;
            }

            std::cout << "Status:"
                      << " state=" << exec_state_to_string(res.state())
                      << " connected=" << res.connected()
                      << " armed=" << res.armed()
                      << " in_air=" << res.in_air()
                      << " relative_altitude_m=" << res.relative_altitude_m()
                      << " flight_mode=" << flight_mode_to_string(res.flight_mode())
                      << " queue_size=" << res.queue_size()
                      << " last_error=" << res.last_error()
                      << "\n";
            continue;
        }

        if (cmd == "hover") {
            double seconds{};
            if (!(iss >> seconds)) {
                std::cout << "Usage: hover <seconds>\n";
                continue;
            }

            drone::Command req;
            req.set_id(std::to_string(next_id++));
            req.mutable_hover()->set_seconds(seconds);

            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub->Enqueue(&ctx, req, &ack);

            std::cout << "Enqueue grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " accepted=" << ack.accepted()
                      << " exec_state=" << exec_state_to_string(ack.state())
                      << " command_state=" << command_state_to_string(ack.command_state())
                      << " reply_msg=" << ack.message()
                      << "\n";

            if (status.ok() && ack.accepted()) {
                watcher_threads.emplace_back([addr, id = ack.id()]() {
                    watch_command(addr, id);
                });
            }
            continue;
        }

        if (cmd == "fly") {
            double seconds{};
            double velocity{};
            if (!(iss >> seconds >> velocity)) {
                std::cout << "Usage: fly <seconds> <velocity_m_s>\n";
                continue;
            }

            drone::Command req;
            req.set_id(std::to_string(next_id++));
            auto* fly = req.mutable_flyforward();
            fly->set_seconds(seconds);
            fly->set_velocity(velocity);

            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub->Enqueue(&ctx, req, &ack);

            std::cout << "Enqueue grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " accepted=" << ack.accepted()
                      << " exec_state=" << exec_state_to_string(ack.state())
                      << " command_state=" << command_state_to_string(ack.command_state())
                      << " reply_msg=" << ack.message()
                      << "\n";

            if (status.ok() && ack.accepted()) {
                watcher_threads.emplace_back([addr, id = ack.id()]() {
                    watch_command(addr, id);
                });
            }
            continue;
        }

        if (cmd == "turn") {
            double degrees{};
            double seconds{};
            if (!(iss >> degrees >> seconds)) {
                std::cout << "Usage: turn <degrees> <seconds>\n";
                continue;
            }

            drone::Command req;
            req.set_id(std::to_string(next_id++));
            auto* turn = req.mutable_turnyaw();
            turn->set_deg(degrees);
            turn->set_seconds(seconds);

            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub->Enqueue(&ctx, req, &ack);

            std::cout << "Enqueue grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " accepted=" << ack.accepted()
                      << " exec_state=" << exec_state_to_string(ack.state())
                      << " command_state=" << command_state_to_string(ack.command_state())
                      << " reply_msg=" << ack.message()
                      << "\n";

            if (status.ok() && ack.accepted()) {
                watcher_threads.emplace_back([addr, id = ack.id()]() {
                    watch_command(addr, id);
                });
            }
            continue;
        }

        if (cmd == "land") {
            drone::Command req;
            req.set_id(std::to_string(next_id++));
            req.mutable_land();

            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub->Enqueue(&ctx, req, &ack);

            std::cout << "Enqueue grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " accepted=" << ack.accepted()
                      << " exec_state=" << exec_state_to_string(ack.state())
                      << " command_state=" << command_state_to_string(ack.command_state())
                      << " reply_msg=" << ack.message()
                      << "\n";

            if (status.ok() && ack.accepted()) {
                watcher_threads.emplace_back([addr, id = ack.id()]() {
                    watch_command(addr, id);
                });
            }
            continue;
        }

        std::cout << "Unknown command: " << cmd << " (type 'help')\n";
    }

    std::cout << "Bye.\n";

    for (auto& t : watcher_threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    return 0;
}