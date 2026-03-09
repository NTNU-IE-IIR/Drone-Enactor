#pragma once

#include <grpcpp/grpcpp.h>
#include "drone_control.grpc.pb.h"
#include "flight_controller.h"

class DroneControlService final : public drone::DroneControl::Service {
public:
    explicit DroneControlService(FlightController& controller);

    grpc::Status Enqueue(grpc::ServerContext*,
                         const drone::Command* req,
                         drone::CommandAck* reply) override;

    grpc::Status StopNow(grpc::ServerContext*,
                         const drone::StopRequest* req,
                         drone::StopReply* reply) override;

private:
    FlightController& _controller;
};