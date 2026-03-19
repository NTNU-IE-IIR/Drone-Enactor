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

    grpc::Status GetStatus(grpc::ServerContext*,
                           const google::protobuf::Empty* req,
                           drone::StatusReply* reply) override;

private:
    static drone::ExecState to_proto_state(FlightController::ExecState state);

    FlightController& _controller;
};