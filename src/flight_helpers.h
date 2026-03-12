#pragma once

#include <chrono>
#include <string>

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "drone_control.pb.h"
#include <mavsdk/plugins/offboard/offboard.h>

const char* action_result_to_string(mavsdk::Action::Result r);
const char* offboard_result_to_string(mavsdk::Offboard::Result r);


bool ensure_airborne_or_reply(
    mavsdk::Action& action,
    mavsdk::Telemetry& telemetry,
    drone::CommandAck* reply);

bool wait_until_ready_for_offboard_or_reply(
    mavsdk::Telemetry& telemetry,
    drone::CommandAck* reply);


const char* action_result_to_string(mavsdk::Action::Result r);
