#include "drone_control_service.h"
#include <google/protobuf/empty.pb.h>

DroneControlService::DroneControlService(FlightController& controller)
    : _controller(controller)
{
}

drone::ExecState DroneControlService::to_proto_state(FlightController::ExecState state)
{
    switch (state) {
        case FlightController::ExecState::Disconnected:
            return drone::DISCONNECTED;
        case FlightController::ExecState::Idle:
            return drone::IDLE;
        case FlightController::ExecState::Arming:
            return drone::ARMING;
        case FlightController::ExecState::TakingOff:
            return drone::TAKINGOFF;
        case FlightController::ExecState::Hovering:
            return drone::HOVERING;
        case FlightController::ExecState::ExecutingCommand:
            return drone::EXECUTINGCOMMAND;
        case FlightController::ExecState::Landing:
            return drone::LANDING;
        case FlightController::ExecState::EmergencyStopped:
            return drone::EMERGENCYSTOPPED;
        case FlightController::ExecState::Error:
            return drone::ERROR;
    }

    return drone::ERROR;
}

drone::CommandState DroneControlService::to_proto_command_state(FlightController::CommandState state)
{
    switch (state) {
        case FlightController::CommandState::Unknown:
            return drone::COMMAND_STATE_UNKNOWN;
        case FlightController::CommandState::Queued:
            return drone::COMMAND_STATE_QUEUED;
        case FlightController::CommandState::Running:
            return drone::COMMAND_STATE_RUNNING;
        case FlightController::CommandState::Succeeded:
            return drone::COMMAND_STATE_SUCCEEDED;
        case FlightController::CommandState::Failed:
            return drone::COMMAND_STATE_FAILED;
        case FlightController::CommandState::Interrupted:
            return drone::COMMAND_STATE_INTERRUPTED;
        case FlightController::CommandState::Cancelled:
            return drone::COMMAND_STATE_CANCELLED;
    }

    return drone::COMMAND_STATE_UNKNOWN;
}

grpc::Status DroneControlService::Enqueue(grpc::ServerContext*,
                                          const drone::Command* req,
                                          drone::CommandAck* reply)
{
    reply->set_id(req->id());

    bool ok = false;

    if (req->has_hover()) {
        ok = _controller.enqueue_hover(req->id(), req->hover().seconds());
    } else if (req->has_flyforward()) {
        ok = _controller.enqueue_fly_forward(
            req->id(),
            req->flyforward().seconds(),
            req->flyforward().velocity());
    } else if (req->has_turnyaw()) {
        ok = _controller.enqueue_turn_yaw(
            req->id(),
            req->turnyaw().deg(),
            req->turnyaw().seconds());
    } else if (req->has_land()) {
        ok = _controller.enqueue_land(req->id());
    } else {
        reply->set_accepted(false);
        reply->set_message("Unsupported command payload");
        reply->set_state(to_proto_state(_controller.current_state()));
        reply->set_command_state(drone::COMMAND_STATE_FAILED);
        return grpc::Status::OK;
    }

    reply->set_accepted(ok);
    reply->set_message(ok ? "Queued" : _controller.last_error());
    reply->set_state(to_proto_state(_controller.current_state()));
    reply->set_command_state(ok ? drone::COMMAND_STATE_QUEUED
                                : drone::COMMAND_STATE_FAILED);
    return grpc::Status::OK;
}

grpc::Status DroneControlService::StopNow(grpc::ServerContext*,
                                          const drone::StopRequest*,
                                          drone::StopReply* reply)
{
    const bool ok = _controller.request_stop();
    reply->set_ok(ok);
    reply->set_message(ok ? "Emergency stop executed" : _controller.last_error());
    reply->set_state(to_proto_state(_controller.current_state()));
    return grpc::Status::OK;
}

grpc::Status DroneControlService::GetStatus(grpc::ServerContext*,
                                            const google::protobuf::Empty*,
                                            drone::StatusReply* reply)
{
    const auto status = _controller.get_status();

    reply->set_state(to_proto_state(status.state));
    reply->set_last_error(status.last_error);
    reply->set_connected(status.connected);
    reply->set_armed(status.armed);
    reply->set_in_air(status.in_air);
    reply->set_relative_altitude_m(status.relative_altitude_m);
    reply->set_flight_mode(status.flight_mode);
    reply->set_queue_size(status.queue_size);

    return grpc::Status::OK;
}

grpc::Status DroneControlService::GetCommandStatus(grpc::ServerContext*,
                                                   const drone::CommandStatusRequest* req,
                                                   drone::CommandStatusReply* reply)
{
    const auto status = _controller.get_command_status(req->id());

    reply->set_id(req->id());
    reply->set_found(status.found);
    reply->set_command_state(to_proto_command_state(status.state));
    reply->set_message(status.message);
    reply->set_controller_state(to_proto_state(_controller.current_state()));

    return grpc::Status::OK;
}