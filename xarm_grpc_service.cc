#include <CLI11/include/CLI/CLI.hpp>
#include <iostream>
#include <memory>
#include <string>
#define FMT_HEADER_ONLY
#include <fmt/include/fmt/core.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <xarm/wrapper/xarm_api.h>

#include "xapi.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using xapi::Empty;
using xapi::InitParam;
using xapi::Mode;
using xapi::MotionEnable;
using xapi::Position;
using xapi::State;
using xapi::Version;
using xapi::XAPI;

// XAPI server
class XAPIServiceImpl final : public XAPI::Service {
    // ===== Connection specific =====
    Status Disconnect(ServerContext* context, const Empty* empty1,
                      Empty* empty2) override {
        api->disconnect();
        return Status::OK;
    }

    Status Initialize(ServerContext* context, const InitParam* p,
                      Empty* empty) override {
        api = new XArmAPI(p->ip_address());
        return Status::OK;
    }
    // ===== Properties =====
    Status GetVersion(ServerContext* context, const Empty* empty,
                      Version* version) override {
        int status_code;
        unsigned char version_char[40];
        status_code = api->get_version(version_char);
        std::string version_str(
            version_char,
            version_char + sizeof version_char / sizeof version_char[0]);
        version->set_version(version_str);
        version->set_status_code(status_code);
        return Status::OK;
    }

    Status GetState(ServerContext* context, const Empty* empty,
                    State* state) override {
        int status_code_tmp;
        int state_tmp;
        status_code_tmp = api->get_state(&state_tmp);
        state->set_state(state_tmp);
        state->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status SetState(ServerContext* context, const State* state,
                    State* state_res) override {
        int status_code_tmp;
        status_code_tmp = api->set_state(state->state());
        state_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status SetMode(ServerContext* context, const Mode* mode,
                   Mode* mode_res) override {
        int status_code_tmp;
        status_code_tmp = api->set_mode(mode->mode());
        mode_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetPosition(ServerContext* context, const Empty* empty,
                       Position* position) override {
        int status_code_tmp;
        fp32 pose[6];
        status_code_tmp = api->get_position(pose);
        position->set_x(pose[0]);
        position->set_y(pose[1]);
        position->set_z(pose[2]);
        position->set_roll(pose[3]);
        position->set_pitch(pose[4]);
        position->set_yaw(pose[5]);
        position->set_status_code(status_code_tmp);
        return Status::OK;
    }

    // ===== Actions =====

    Status SetMotionEnable(ServerContext* context,
                           const MotionEnable* motion_enable,
                           MotionEnable* motion_enable_res) override {
        int status_code_tmp;
        status_code_tmp = api->motion_enable(motion_enable->enable(),
                                             motion_enable->servo_id());
        motion_enable_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status SetPosition(ServerContext* context, const Position* position,
                       Position* position_res) override {
        int status_code_tmp;
        fp32 pose[6];
        pose[0] = position->x();
        pose[1] = position->y();
        pose[2] = position->z();
        pose[3] = position->roll();
        pose[4] = position->yaw();
        pose[5] = position->pitch();
        bool wait = position->wait();

        status_code_tmp = api->set_position(pose, wait);

        position_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    XArmAPI* api;
};

void RunServer(std::string server_address) {
    XAPIServiceImpl service;

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char** argv) {
    // CLI11 xarm-grpc-service app
    CLI::App app{
        "xarm-grpc-service: a CLI tool for starting the C++ xArmAPI gRPC "
        "service."};

    // Prints defaults on help
    app.option_defaults()->always_capture_default();

    // Options
    std::string server_ip = "127.0.0.1";
    app.add_option("-i, --ip", server_ip, "IP address of the gRPC server");
    std::string server_port = "50051";
    app.add_option("-p, --port", server_port, "port of the gRPC server");

    // Subcommands
    app.require_subcommand(1);  // set max number of subcommands to 1

    // Subcommand: start
    auto* start = app.add_subcommand("start", "start the service");
    start->callback(
        [&]() { RunServer(fmt::format("{}:{}", server_ip, server_port)); });

    CLI11_PARSE(app, argc, argv);

    return 0;
}
