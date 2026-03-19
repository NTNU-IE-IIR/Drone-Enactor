cmake_minimum_required(VERSION 3.20)
project(BachelorProsjektAPI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(PREPEND CMAKE_PREFIX_PATH
        "/opt/homebrew"
        "/opt/homebrew/opt/abseil"
        "/opt/homebrew/lib/cmake"
)
list(APPEND CMAKE_IGNORE_PREFIX_PATH "/usr/local")
list(APPEND CMAKE_IGNORE_PATH "/usr/local" "/usr/local/include" "/usr/local/lib")


list(PREPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/opt/homebrew/opt/abseil")
set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
set(CMAKE_FIND_USE_PACKAGE_REGISTRY OFF)

find_package(absl CONFIG REQUIRED)


if(NOT TARGET absl::absl_check)
    if(TARGET absl::check)
        add_library(absl::absl_check ALIAS absl::check)
    elseif(TARGET absl::absl_log_internal_check_op)
        add_library(absl::absl_check ALIAS absl::absl_log_internal_check_op)
    else()
        add_library(absl::absl_check INTERFACE IMPORTED)
        if(TARGET absl::base)
            target_link_libraries(absl::absl_check INTERFACE absl::base)
        endif()
    endif()
endif()

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
set(MAVSDK_DIR "/opt/homebrew/lib/cmake/MAVSDK")
find_package(MAVSDK REQUIRED CONFIG)
get_target_property(MAVSDK_INCLUDE_DIRS MAVSDK::mavsdk INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "MAVSDK include dirs: ${MAVSDK_INCLUDE_DIRS}")

set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_FILE "${PROTO_DIR}/drone_control.proto")
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

add_custom_command(
        OUTPUT
        "${GEN_DIR}/drone_control.pb.cc"
        "${GEN_DIR}/drone_control.pb.h"
        "${GEN_DIR}/drone_control.grpc.pb.cc"
        "${GEN_DIR}/drone_control.grpc.pb.h"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${GEN_DIR}"
        COMMAND /opt/homebrew/bin/protoc
        ARGS
        -I "${PROTO_DIR}"
        --cpp_out=${GEN_DIR}
        --grpc_out=${GEN_DIR}
        --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
        "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}"
        VERBATIM
)

add_library(drone_proto
        "${GEN_DIR}/drone_control.pb.cc"
        "${GEN_DIR}/drone_control.grpc.pb.cc"
)
target_include_directories(drone_proto PUBLIC "${GEN_DIR}")
target_link_libraries(drone_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

# ---- Server executable ----
add_executable(drone_api_server
        src/server_main.cpp
        src/drone_control_service.cpp
        src/flight_helpers.cpp
        src/flight_controller.cpp
        src/flight_controller.h
)
target_link_libraries(drone_api_server PRIVATE drone_proto MAVSDK::mavsdk)
add_dependencies(drone_api_server drone_proto)

# ---- Client demo executable ----
add_executable(drone_client_demo
        src/client_demo.cpp
)
target_link_libraries(drone_client_demo PRIVATE drone_proto gRPC::grpc++)
add_dependencies(drone_client_demo drone_proto)

# ---- Planner CLI executable (separat main) ----
add_executable(planner_cli
        src/planner_cli.cpp
)
target_link_libraries(planner_cli PRIVATE drone_proto gRPC::grpc++)
add_dependencies(planner_cli drone_proto)
