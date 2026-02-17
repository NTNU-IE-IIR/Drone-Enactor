#include <iostream>
#include <grpcpp/grpcpp.h>

#include "drone_control.grpc.pb.h"

int main() {
    auto channel = grpc::CreateChannel("127.0.0.1:60051", grpc::InsecureChannelCredentials());
    auto stub = drone::DroneControl::NewStub(channel);

    drone::Command cmd;
    cmd.set_id("test-1");
    cmd.mutable_hover()->set_seconds(1.0);

    drone::CommandAck ack;
    grpc::ClientContext ctx;
    auto status = stub->Enqueue(&ctx, cmd, &ack);

    std::cout << "Enqueue status=" << status.ok()
              << " accepted=" << ack.accepted()
              << " msg=" << ack.message() << "\n";
    return 0;
}
