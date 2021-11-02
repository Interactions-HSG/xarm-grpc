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
using xapi::Cmdnum;
using xapi::CollisionSensitivity;
using xapi::Empty;
using xapi::InitParam;
using xapi::Mode;
using xapi::MotionEnable;
using xapi::MoveCircleMsg;
using xapi::Position;
using xapi::ResetMsg;
using xapi::ServoAngles;
using xapi::State;
using xapi::TeachSensitivity;
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

    // ===== Read methods =====
    Status GetCollisionSensitivity(
        ServerContext* context, const Empty* empty,
        CollisionSensitivity* collision_sensitivity) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        int collision_sensitivity_tmp = api->collision_sensitivity;
        collision_sensitivity->set_collision_sensitivity(
            collision_sensitivity_tmp);
        collision_sensitivity->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetTeachSensitivity(ServerContext* context, const Empty* empty,
                               TeachSensitivity* teach_sensitivity) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        int teach_sensitivity_tmp = api->teach_sensitivity;
        teach_sensitivity->set_teach_sensitivity(teach_sensitivity_tmp);
        teach_sensitivity->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetVersion(ServerContext* context, const Empty* empty,
                      Version* version) override {
        int status_code;
        unsigned char version_char[40];
        status_code = api->get_version(version_char);
        std::string version_str(
            version_char,
            version_char + sizeof version_char / sizeof version_char[0]);
        version_str.resize(version_str.find('\0'));
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

    Status GetCmdnum(ServerContext* context, const Empty* empty,
                     Cmdnum* cmdnum) override {
        int status_code_tmp;
        int cmdnum_tmp;
        status_code_tmp = api->get_cmdnum(&cmdnum_tmp);
        cmdnum->set_cmdnum(cmdnum_tmp);
        cmdnum->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetServoAngles(ServerContext* context, const Empty* empty,
                          ServoAngles* servo_angles) override {
        int status_code_tmp;
        fp32 angles[7];
        status_code_tmp = api->get_servo_angle(angles);
        servo_angles->set_servo_1(angles[0]);
        servo_angles->set_servo_2(angles[1]);
        servo_angles->set_servo_3(angles[2]);
        servo_angles->set_servo_4(angles[3]);
        servo_angles->set_servo_5(angles[4]);
        servo_angles->set_servo_6(angles[5]);
        servo_angles->set_servo_7(angles[6]);
        servo_angles->set_status_code(status_code_tmp);
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

    // ===== Write methods =====
    Status SetMotionEnable(ServerContext* context,
                           const MotionEnable* motion_enable,
                           MotionEnable* motion_enable_res) override {
        int status_code_tmp;
        status_code_tmp = api->motion_enable(motion_enable->enable(),
                                             motion_enable->servo_id());
        motion_enable_res->set_status_code(status_code_tmp);
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

    Status SetCollisionSensitivity(
        ServerContext* context,
        const CollisionSensitivity* collision_sensitivity,
        CollisionSensitivity* collision_sensitivity_res) override {
        int status_code_tmp;
        status_code_tmp = api->set_collision_sensitivity(
            collision_sensitivity->collision_sensitivity());
        collision_sensitivity_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status SetTeachSensitivity(
        ServerContext* context, const TeachSensitivity* teach_sensitivity,
        TeachSensitivity* teach_sensitivity_res) override {
        int status_code_tmp;
        status_code_tmp =
            api->set_teach_sensitivity(teach_sensitivity->teach_sensitivity());
        teach_sensitivity_res->set_status_code(status_code_tmp);
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

    Status SetServoAngles(ServerContext* context,
                          const ServoAngles* servo_angles,
                          ServoAngles* servo_angles_res) override {
        int status_code_tmp;
        fp32 angles[6];
        angles[0] = servo_angles->servo_1();
        angles[1] = servo_angles->servo_2();
        angles[2] = servo_angles->servo_3();
        angles[3] = servo_angles->servo_4();
        angles[4] = servo_angles->servo_5();
        angles[5] = servo_angles->servo_6();
        angles[6] = servo_angles->servo_7();

        bool wait = servo_angles->wait();

        status_code_tmp = api->set_servo_angle(angles, wait);

        servo_angles_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status MoveCircle(ServerContext* context, const MoveCircleMsg* move_circle,
                      MoveCircleMsg* move_circle_res) override {
        int status_code_tmp;
        fp32 pose_1[6];
        pose_1[0] = move_circle->pose_1().x();
        pose_1[1] = move_circle->pose_1().y();
        pose_1[2] = move_circle->pose_1().z();
        pose_1[3] = move_circle->pose_1().roll();
        pose_1[4] = move_circle->pose_1().yaw();
        pose_1[5] = move_circle->pose_1().pitch();

        fp32 pose_2[6];
        pose_2[0] = move_circle->pose_2().x();
        pose_2[1] = move_circle->pose_2().y();
        pose_2[2] = move_circle->pose_2().z();
        pose_2[3] = move_circle->pose_2().roll();
        pose_2[4] = move_circle->pose_2().yaw();
        pose_2[5] = move_circle->pose_2().pitch();

        std::cout << "x1 = " << pose_1[0] << "y1 = " << pose_1[1]
                  << ", z1 = " << pose_1[2] << "\n";
        std::cout << "x2 = " << pose_2[0] << "y2 = " << pose_2[1]
                  << ", z2 = " << pose_1[2] << "\n";

        fp32 percent = move_circle->percent();
        fp32 speed = move_circle->speed();
        fp32 acc = move_circle->acc();

        bool wait = move_circle->wait();
        fp32 timeout = move_circle->timeout();

        // @param mvtime: 0, reserved
        status_code_tmp = api->move_circle(pose_1, pose_2, percent, speed, acc,
                                           0, wait, timeout);

        move_circle_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status Reset(ServerContext* context, const ResetMsg* reset,
                 Empty* empty) override {
        api->reset(reset->wait(), reset->timeout());
        return Status::OK;
    }

    Status EmergencyStop(ServerContext* context, const Empty* empty1,
                         Empty* empty2) override {
        api->emergency_stop();
        return Status::OK;
    }


    Status GetInverseKinematics(ServerContext* context, const Position* position,
                       ServoAngles* servo_angles) override {
        int status_code_tmp;
        fp32 pose[6];
        pose[0] = position->x();
        pose[1] = position->y();
        pose[2] = position->z();
        pose[3] = position->roll();
        pose[4] = position->yaw();
        pose[5] = position->pitch();

        fp32 angles[7];
        status_code_tmp = api->get_inverse_kinematics(pose, angles);

        servo_angles->set_servo_1(angles[0]);
        servo_angles->set_servo_2(angles[1]);
        servo_angles->set_servo_3(angles[2]);
        servo_angles->set_servo_4(angles[3]);
        servo_angles->set_servo_5(angles[4]);
        servo_angles->set_servo_6(angles[5]);
        servo_angles->set_servo_7(angles[6]);
        servo_angles->set_status_code(status_code_tmp);

        return Status::OK;
    }

    Status GetForwardKinematics(ServerContext* context,
                          const ServoAngles* servo_angles,
                          Position* position) override {
        int status_code_tmp;
        fp32 angles[6];
        angles[0] = servo_angles->servo_1();
        angles[1] = servo_angles->servo_2();
        angles[2] = servo_angles->servo_3();
        angles[3] = servo_angles->servo_4();
        angles[4] = servo_angles->servo_5();
        angles[5] = servo_angles->servo_6();
        angles[6] = servo_angles->servo_7();

        fp32 pose[6];
        status_code_tmp = api->get_forward_kinematics(angles, pose);

        position->set_x(pose[0]);
        position->set_y(pose[1]);
        position->set_z(pose[2]);
        position->set_roll(pose[3]);
        position->set_pitch(pose[4]);
        position->set_yaw(pose[5]);
        position->set_status_code(status_code_tmp);
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
