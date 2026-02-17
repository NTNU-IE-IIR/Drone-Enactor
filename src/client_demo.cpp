#include <iostream>
#include <grpcpp/grpcpp.h>

#include "drone_control.grpc.pb.h"

int main() {
    auto channel = grpc::CreateChannel("127.0.0.1:60051", grpc::InsecureChannelCredentials());
    auto stub = drone::DroneControl::NewStub(channel);

    drone::Command cmd;
    cmd.set_id("test-1");
    cmd.mutable_hover()->set_seconds(10.0);
    auto* fly = cmd.mutable_flyforward();
    fly ->set_seconds(10);
    fly -> set_velocity(-10);

    drone::CommandAck ack;
    grpc::ClientContext ctx;
    auto status = stub->Enqueue(&ctx, cmd, &ack);

    std::cout << "rpc_ok=" << status.ok()
          << " code=" << status.error_code()
          << " msg=" << status.error_message()
          << " accepted=" << ack.accepted()
          << " reply_msg=" << ack.message()
          << "\n";
    return 0;
}
