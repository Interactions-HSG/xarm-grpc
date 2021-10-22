#include <iostream>
#include <memory>
#include <string>

#include <CLI11/include/CLI/CLI.hpp>
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
using xapi::XAPI;
using xapi::Empty;
using xapi::InitParam;
using xapi::Version;

// XAPI server
class XAPIServiceImpl final : public XAPI::Service {
  Status Disconnect(ServerContext* context, const Empty* empty1, Empty* empty2) override {
    api->disconnect();
    return Status::OK;
  }

  Status GetVersion(ServerContext* context, const Empty* empty, Version* version) override {
    int status_code;
    unsigned char version_char[40];
    status_code = api->get_version(version_char);
    std::string version_str(version_char, version_char + sizeof version_char / sizeof version_char[0]);
    version->set_version(version_str);
    version->set_status_code(status_code);
    return Status::OK;
  }

  Status Initialize(ServerContext* context, const InitParam* p, Empty* empty) override {
    api = new XArmAPI(p->ip_address());
    return Status::OK;
  }

  XArmAPI *api;
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

int main(int argc, char **argv) {
    // CLI11 xarm-grpc-service app
    CLI::App app{"xarm-grpc-service: a CLI tool for starting the C++ xArmAPI gRPC service."};

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
    auto *start =
        app.add_subcommand("start", "start the service");
    start->callback([&]() {
        RunServer(fmt::format("{}:{}", server_ip, server_port));
    });

    CLI11_PARSE(app, argc, argv);

    return 0;
}
