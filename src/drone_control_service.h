#pragma once

#include <memory>

#include <grpcpp/grpcpp.h>

#include <mavsdk/system.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "drone_control.grpc.pb.h"

class DroneControlService final : public drone::DroneControl::Service {
public:
    DroneControlService(mavsdk::System& system,
                        mavsdk::Action& action,
                        mavsdk::Telemetry& telemetry);

    grpc::Status Enqueue(grpc::ServerContext*,
                         const drone::Command* req,
                         drone::CommandAck* reply) override;

    grpc::Status StopNow(grpc::ServerContext*,
                         const drone::StopRequest*,
                         drone::StopReply* reply) override;

private:
    mavsdk::System& _system;
    mavsdk::Action& _action;
    mavsdk::Telemetry& _telemetry;

    grpc::Status handle_hover(const drone::Hover& hover, drone::CommandAck* reply);
    grpc::Status handle_flyforward(const drone::FlyForward& fly, drone::CommandAck* reply);
    grpc::Status handle_turn_yaw(const drone::TurnYaw& turn, drone::CommandAck* reply);
    grpc::Status handle_land(const drone::Land& land, drone::CommandAck* reply);
};
