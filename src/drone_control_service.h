#pragma once
#include <grpcpp/grpcpp.h>
#include "drone_control.grpc.pb.h"

class FlightController;

class DroneControlService final : public drone::DroneControl::Service {
public:
    explicit DroneControlService(FlightController& fc) : _fc(fc) {}

    grpc::Status Enqueue(grpc::ServerContext*,
                         const drone::Command* req,
                         drone::CommandAck* reply) override;

    grpc::Status StopNow(grpc::ServerContext*,
                         const drone::StopRequest*,
                         drone::StopReply* reply) override;

private:
    FlightController& _fc;
};