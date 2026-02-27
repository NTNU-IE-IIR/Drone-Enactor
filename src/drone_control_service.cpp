#include "drone_control_service.h"
#include "flight_controller.h"

#include <iostream>

grpc::Status DroneControlService::Enqueue(grpc::ServerContext*,
                                         const drone::Command* req,
                                         drone::CommandAck* reply)
{
    reply->set_id(req->id());

    bool ok = false;

    if (req->has_hover()) {
        ok = _fc.request_hover(req->hover().seconds());
    } else if (std::cout << "[API] Enqueue called. has_flyforward=" << req->has_flyforward() << "\n") {
        ok = _fc.request_fly_forward(req->flyforward().seconds(), req->flyforward().velocity());
    } else if (req->has_turnyaw()) {
        ok = _fc.request_turn_yaw(req->turnyaw().deg(), req->turnyaw().seconds());
    } else if (req->has_land()) {
        ok = _fc.request_land();
    } else {
        reply->set_accepted(false);
        reply->set_message("Unsupported command payload");
        return grpc::Status::OK;
    }

    reply->set_accepted(ok);
    reply->set_message(ok ? "Accepted" : _fc.last_error());
    return grpc::Status::OK;
}

grpc::Status DroneControlService::StopNow(grpc::ServerContext*,
                                         const drone::StopRequest*,
                                         drone::StopReply* reply)
{
    const bool ok = _fc.request_stop();
    reply->set_ok(ok);
    reply->set_message(ok ? "Stopped (zero cmd)" : _fc.last_error());
    return grpc::Status::OK;
}
