#include "drone_control_service.h"
#include "flight_controller.h"

DroneControlService::DroneControlService(FlightController& controller)
    : _controller(controller)
{
}

grpc::Status DroneControlService::Enqueue(grpc::ServerContext*,
                                          const drone::Command* req,
                                          drone::CommandAck* reply)
{
    reply->set_id(req->id());

    bool ok = false;

    if (req->has_hover()) {
        ok = _controller.enqueue_hover(req->hover().seconds());
    } else if (req->has_flyforward()) {
        ok = _controller.enqueue_fly_forward(
            req->flyforward().seconds(),
            req->flyforward().velocity()
        );
    } else if (req->has_turnyaw()) {
        ok = _controller.enqueue_turn_yaw(
            req->turnyaw().deg(),
            req->turnyaw().seconds()
        );
    } else if (req->has_land()) {
        ok = _controller.enqueue_land();
    } else {
        reply->set_accepted(false);
        reply->set_message("Unsupported command payload");
        return grpc::Status::OK;
    }

    reply->set_accepted(ok);
    reply->set_message(ok ? "Queued" : _controller.last_error());
    return grpc::Status::OK;
}

grpc::Status DroneControlService::StopNow(grpc::ServerContext*,
                                          const drone::StopRequest*,
                                          drone::StopReply* reply)
{
    const bool ok = _controller.request_stop();
    reply->set_ok(ok);
    reply->set_message(ok ? "Stopped immediately" : _controller.last_error());
    return grpc::Status::OK;
}
