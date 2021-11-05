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
using xapi::Currents;
using xapi::DefaultIsRadian;
using xapi::Empty;
using xapi::InitParam;
using xapi::Mode;
using xapi::MotionEnable;
using xapi::MoveCircleMsg;
using xapi::PauseTime;
using xapi::Position;
using xapi::ResetMsg;
using xapi::RobotSN;
using xapi::ServoAngle;
using xapi::ServoAngles;
using xapi::SetPositionMsg;
using xapi::SetServoAngleMsg;
using xapi::SimulationRobot;
using xapi::State;
using xapi::TeachSensitivity;
using xapi::Temperatures;
using xapi::Version;
using xapi::Voltages;
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

    Status GetMode(ServerContext* context, const Empty* empty,
                   Mode* mode) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        int mode_tmp = api->mode;
        mode->set_mode(mode_tmp);
        mode->set_status_code(status_code_tmp);
        return Status::OK;
    }

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

    Status GetLastUsedAngles(ServerContext* context, const Empty* empty,
                             ServoAngles* servo_angles) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        fp32* angles;
        angles = api->last_used_angles;
        servo_angles->set_servo_1(*(angles));
        servo_angles->set_servo_2(*(angles + 1));
        servo_angles->set_servo_3(*(angles + 2));
        servo_angles->set_servo_4(*(angles + 3));
        servo_angles->set_servo_5(*(angles + 4));
        servo_angles->set_servo_6(*(angles + 5));
        servo_angles->set_servo_7(*(angles + 6));
        servo_angles->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetTemperatures(ServerContext* context, const Empty* empty,
                           Temperatures* temperatures) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        fp32* temp;
        temp = api->temperatures;
        temperatures->set_servo_1(*(temp));
        temperatures->set_servo_2(*(temp + 1));
        temperatures->set_servo_3(*(temp + 2));
        temperatures->set_servo_4(*(temp + 3));
        temperatures->set_servo_5(*(temp + 4));
        temperatures->set_servo_6(*(temp + 5));
        temperatures->set_servo_7(*(temp + 6));
        temperatures->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetDefaultIsRadian(ServerContext* context, const Empty* empty,
                              DefaultIsRadian* default_is_radian) override {
        int status_code_tmp = 0;
        bool default_is_radian_tmp = api->default_is_radian;
        default_is_radian->set_default_is_radian(default_is_radian_tmp);
        default_is_radian->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetVoltages(ServerContext* context, const Empty* empty,
                       Voltages* voltages) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        fp32* volt;
        volt = api->voltages;
        voltages->set_servo_1(*(volt));
        voltages->set_servo_2(*(volt + 1));
        voltages->set_servo_3(*(volt + 2));
        voltages->set_servo_4(*(volt + 3));
        voltages->set_servo_5(*(volt + 4));
        voltages->set_servo_6(*(volt + 5));
        voltages->set_servo_7(*(volt + 6));
        voltages->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetCurrents(ServerContext* context, const Empty* empty,
                       Currents* currents) override {
        int status_code_tmp = 0;  // TODO(jo-bru): status code for properties
        fp32* curr;
        curr = api->currents;
        currents->set_servo_1(*(curr));
        currents->set_servo_2(*(curr + 1));
        currents->set_servo_3(*(curr + 2));
        currents->set_servo_4(*(curr + 3));
        currents->set_servo_5(*(curr + 4));
        currents->set_servo_6(*(curr + 5));
        currents->set_servo_7(*(curr + 6));
        currents->set_status_code(status_code_tmp);
        currents->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status GetSimulationRobot(ServerContext* context, const Empty* empty,
                              SimulationRobot* simulation_robot) override {
        int status_code_tmp = 0;
        bool is_simulation_robot = api->is_simulation_robot;
        simulation_robot->set_on(is_simulation_robot);
        simulation_robot->set_status_code(status_code_tmp);
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

    Status GetRobotSN(ServerContext* context, const Empty* empty,
                      RobotSN* robot_sn) override {
        int status_code;
        unsigned char robot_sn_char[40];
        status_code = api->get_robot_sn(robot_sn_char);
        std::string robot_sn_str(
            robot_sn_char,
            robot_sn_char + sizeof robot_sn_char / sizeof robot_sn_char[0]);
        robot_sn_str.resize(robot_sn_str.find('\0'));
        robot_sn->set_robot_sn(robot_sn_str);
        robot_sn->set_status_code(status_code);
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
    Status SetDefaultIsRadian(ServerContext* context,
                              const DefaultIsRadian* default_is_radian,
                              DefaultIsRadian* default_is_radian_res) override {
        int status_code_tmp = 0;
        api->default_is_radian = default_is_radian->default_is_radian();
        default_is_radian_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

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

    Status SetPauseTime(ServerContext* context, const PauseTime* pause_time,
                        PauseTime* pause_time_res) override {
        int status_code_tmp;
        status_code_tmp = api->set_pause_time(pause_time->sltime());
        pause_time_res->set_status_code(status_code_tmp);
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

    Status SetPosition(ServerContext* context,
                       const SetPositionMsg* set_position_msg,
                       SetPositionMsg* set_position_msg_res) override {
        int status_code_tmp;
        fp32 pose[6];
        pose[0] = set_position_msg->pose().x();
        pose[1] = set_position_msg->pose().y();
        pose[2] = set_position_msg->pose().z();
        pose[3] = set_position_msg->pose().roll();
        pose[4] = set_position_msg->pose().yaw();
        pose[5] = set_position_msg->pose().pitch();

        fp32 radius =
            set_position_msg->radius();  // TODO(jo-bru): handle default value
                                         // => when not set => default: -1 (use
                                         // optional or singleton oneof)
        fp32 speed = set_position_msg->speed();
        fp32 acc = set_position_msg->acc();
        bool wait = set_position_msg->wait();
        fp32 timeout =
            set_position_msg->timeout();  // TODO(jo-bru): handle default value
                                          // => when not set => default: -1 (use
                                          // optional or singleton oneof)

        status_code_tmp =
            api->set_position(pose, radius, speed, acc, 0, wait, timeout);

        set_position_msg_res->set_status_code(status_code_tmp);
        return Status::OK;
    }

    Status SetServoAngle(ServerContext* context,
                         const SetServoAngleMsg* set_servo_angle_msg,
                         SetServoAngleMsg* set_servo_angle_msg_res) override {
        int status_code_tmp;

        fp32 speed = set_servo_angle_msg->speed();
        fp32 acc = set_servo_angle_msg->acc();
        bool wait = set_servo_angle_msg->wait();
        fp32 timeout = set_servo_angle_msg
                           ->timeout();  // TODO(jo-bru): handle default value
                                         // => when not set => default: -1 (use
                                         // optional or singleton oneof)
        fp32 radius = set_servo_angle_msg
                          ->radius();  // TODO(jo-bru): handle default value
                                       // => when not set => default: -1 (use
                                       // optional or singleton oneof)

        // oneof: single angle or all angles
        if (set_servo_angle_msg->has_servo_angle()) {
            std::cout << "Singe angle.."
                      << "\n";
            int servo_id = set_servo_angle_msg->servo_angle().servo_id();
            fp32 angle = set_servo_angle_msg->servo_angle().angle();

            status_code_tmp = api->set_servo_angle(servo_id, angle, speed, acc,
                                                   0, wait, timeout, wait);
        } else {
            fp32 angles[7];
            angles[0] = set_servo_angle_msg->servo_angles().servo_1();
            angles[1] = set_servo_angle_msg->servo_angles().servo_2();
            angles[2] = set_servo_angle_msg->servo_angles().servo_3();
            angles[3] = set_servo_angle_msg->servo_angles().servo_4();
            angles[4] = set_servo_angle_msg->servo_angles().servo_5();
            angles[5] = set_servo_angle_msg->servo_angles().servo_6();
            angles[6] = set_servo_angle_msg->servo_angles().servo_7();

            status_code_tmp = api->set_servo_angle(angles, speed, acc, 0, wait,
                                                   timeout, wait);
        }

        set_servo_angle_msg_res->set_status_code(status_code_tmp);
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

        fp32 percent = move_circle->percent();
        fp32 speed = move_circle->speed();
        fp32 acc = move_circle->acc();

        bool wait = move_circle->wait();
        fp32 timeout =
            move_circle->timeout();  // TODO(jo-bru): handle default value =>
                                     // when not set => default: -1 (use
                                     // optional or singleton oneof)

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

    Status GetInverseKinematics(ServerContext* context,
                                const Position* position,
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

    Status SetSimulationRobot(ServerContext* context,
                              const SimulationRobot* simulation_robot,
                              SimulationRobot* simulation_robot_res) override {
        int status_code_tmp;
        status_code_tmp = api->set_simulation_robot(simulation_robot->on());
        simulation_robot_res->set_status_code(status_code_tmp);
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
