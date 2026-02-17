#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <grpcpp/grpcpp.h>
#include "drone_control.grpc.pb.h"

static void print_help()
{
    std::cout <<
        "Commands:\n"
        "  hover <seconds>\n"
        "  fly <seconds> <velocity_m_s>\n"
        "  turn <degrees> <seconds>\n"
        "  land\n"
        "  stop\n"
        "  help\n"
        "  quit\n";
}

int main(int argc, char** argv)
{

    std::string addr = "127.0.0.1:60051";
    if (argc >= 2) addr = argv[1];

    auto channel = grpc::CreateChannel(addr, grpc::InsecureChannelCredentials());
    std::unique_ptr<drone::DroneControl::Stub> stub = drone::DroneControl::NewStub(channel);

    std::cout << "Planner CLI connected to " << addr << "\n";
    print_help();

    uint64_t next_id = 1;

    for (std::string line; std::cout << "> " && std::getline(std::cin, line);) {
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd.empty()) continue;

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
                      << " reply_msg=" << reply.message()
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

            auto* hover = req.mutable_hover();
            hover->set_seconds(seconds);

            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub->Enqueue(&ctx, req, &ack);

            std::cout << "Enqueue grpc_ok=" << status.ok()
                      << " code=" << status.error_code()
                      << " msg=" << status.error_message()
                      << " accepted=" << ack.accepted()
                      << " reply_msg=" << ack.message()
                      << "\n";
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
            req.set_id(std::to_string(next_id++));;

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
                      << " reply_msg=" << ack.message()
                      << "\n";
            continue;
        }
        if (cmd == "turn") {
            double seconds{};
            double degrees{};
            if (!(iss >> degrees >> seconds)) {
                std::cout << "Usage: turn <degrees> <seconds>\n";
                continue;
            }
            drone::Command req;
            req.set_id(std::to_string(next_id++));
            auto* turn = req.mutable_turnyaw();
            turn -> set_seconds(seconds);
            turn -> set_deg(degrees);
            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub -> Enqueue(&ctx,req, &ack);
            std::cout << "Enqueue grpc_ok=" << status.ok()
          << " code=" << status.error_code()
          << " msg=" << status.error_message()
          << " accepted=" << ack.accepted()
          << " reply_msg=" << ack.message()
          << "\n";
            continue;
        }
        if (cmd == "land") {
            drone::Command req;
            req.set_id(std::to_string(next_id++));
            auto* turn = req.mutable_land();
            drone::CommandAck ack;
            grpc::ClientContext ctx;
            grpc::Status status = stub -> Enqueue(&ctx,req, &ack);
            std::cout << "Enqueue grpc_ok=" << status.ok()
          << " code=" << status.error_code()
          << " msg=" << status.error_message()
          << " accepted=" << ack.accepted()
          << " reply_msg=" << ack.message()
          << "\n";
            continue;
        }

        std::cout << "Unknown command: " << cmd << " (type 'help')\n";
    }

    std::cout << "Bye.\n";
    return 0;
}
