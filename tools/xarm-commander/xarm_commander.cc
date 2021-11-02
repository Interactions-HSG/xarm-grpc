#define FMT_HEADER_ONLY

#include <easyloggingpp/src/easylogging++.h>
#include <fmt/include/fmt/core.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <CLI11/include/CLI/CLI.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "xapi.grpc.pb.h"

namespace constants {
// Default values (to avoid magic numbers)
// default cartesian position
constexpr float kDefaultPosX = 300;
constexpr float kDefaultPosY = 0;
constexpr float kDefaultPosZ = 200;
constexpr float kDefaultPosRoll = 180;  // [Deg] by default
constexpr float kDefaultPosPitch = 0;   // [Deg] by default
constexpr float kDefaultPosYaw = 0;     // [Deg] by default
// default servo angles
constexpr float kDefaultAng1 = 0;
constexpr float kDefaultAng2 = 0;
constexpr float kDefaultAng3 = 0;
constexpr float kDefaultAng4 = 0;
constexpr float kDefaultAng5 = 0;
constexpr float kDefaultAng6 = 0;
constexpr float kDefaultAng7 = 0;

constexpr int kAllServo = 8;
constexpr int kLogInfo = 1;
constexpr int kLogDebug = 2;
constexpr int kLogEval = 3;
}  // namespace constants

using grpc::Channel;
using grpc::ClientContext;
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
using xapi::SimulationRobot;
using xapi::Version;
using xapi::XAPI;

class XAPIClient {
   public:
    explicit XAPIClient(std::shared_ptr<Channel> channel)
        : stub_(XAPI::NewStub(channel)) {}

    // ===== Connection specific =====

    // Instantiate the xArmAPI object with Modbus/TCP connection
    // @param ip_address: the IP address of xArm -- "port" in the SDK
    void Initialize(const std::string &ip_address) {
        // Data we are sending to the server.
        InitParam p;
        p.set_ip_address(ip_address);

        // Container for the data we expect from the server.
        Empty empty;

        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;

        // The actual RPC.
        Status status = stub_->Initialize(&context, p, &empty);

        // Act upon its status.
        // if (status.ok()) {
        //  return;
        //}
        return;
    }

    // Disconnect the Modbus/TCP connection
    void Disconnect() {
        Empty empty;
        ClientContext context;
        Status status = stub_->Disconnect(&context, empty, &empty);
        return;
    }

    // ===== Read methods =====
    // Get the xArm collision sensitivity
    CollisionSensitivity GetCollisionSensitivity() {
        Empty empty;
        CollisionSensitivity collision_sensitivity;
        ClientContext context;
        Status status = stub_->GetCollisionSensitivity(&context, empty,
                                                       &collision_sensitivity);

        return collision_sensitivity;
    }

    // Get the xArm teach sensitivity
    TeachSensitivity GetTeachSensitivity() {
        Empty empty;
        TeachSensitivity teach_sensitivity;
        ClientContext context;
        Status status =
            stub_->GetTeachSensitivity(&context, empty, &teach_sensitivity);

        return teach_sensitivity;
    }

    SimulationRobot GetSimulationRobot() {
        Empty empty;
        SimulationRobot simulation_robot;
        ClientContext context;
        Status status = stub_->GetSimulationRobot(&context, empty,
                                                       &simulation_robot);

        return simulation_robot;
    }

    // Get the xArm version
    Version GetVersion() {
        Empty empty;
        Version version;
        ClientContext context;
        Status status = stub_->GetVersion(&context, empty, &version);

        // Act upon the status of the actual RPC.
        // if (status.ok()) {
        //  return version;
        //}
        return version;
    }

    // Get the xArm state
    State GetState() {
        Empty empty;
        State state;
        ClientContext context;
        Status status = stub_->GetState(&context, empty, &state);

        // Act upon the status of the actual RPC.
        // if (status.ok()) {
        //  return state;
        //}
        return state;
    }

    // Get the cmd count in cache
    Cmdnum GetCmdnum() {
        Empty empty;
        Cmdnum cmdnum;
        ClientContext context;
        Status status = stub_->GetCmdnum(&context, empty, &cmdnum);

        return cmdnum;
    }

    // Get the xArm position
    Position GetPosition() {
        Empty empty;
        Position position;
        ClientContext context;
        Status status = stub_->GetPosition(&context, empty, &position);
        return position;
    }

    // Get the xArm servo angles
    ServoAngles GetServoAngles() {
        Empty empty;
        ServoAngles servo_angles;
        ClientContext context;
        Status status = stub_->GetServoAngles(&context, empty, &servo_angles);
        return servo_angles;
    }

    // ===== Write methods =====

    // Enable the motion of the xArm (specific joints)
    MotionEnable SetMotionEnable(const MotionEnable &motion_enable) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        MotionEnable motion_enable_res;

        Status status =
            stub_->SetMotionEnable(&context, motion_enable, &motion_enable_res);

        return motion_enable_res;
    }

    // Set the xArm state
    State SetState(const State &state) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        State state_res;

        Status status = stub_->SetState(&context, state, &state_res);

        return state_res;
    }

    // Set the xArm mode
    Mode SetMode(const Mode &mode) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        Mode mode_res;

        Status status = stub_->SetMode(&context, mode, &mode_res);

        return mode_res;
    }

    // Set the xArm collision sensitivity
    CollisionSensitivity SetCollisionSensitivity(
        const CollisionSensitivity &collision_sensitivity) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        CollisionSensitivity collision_sensitivity_res;

        Status status = stub_->SetCollisionSensitivity(
            &context, collision_sensitivity, &collision_sensitivity_res);

        return collision_sensitivity_res;
    }

    // Set the xArm teach sensitivity
    TeachSensitivity SetTeachSensitivity(
        const TeachSensitivity &teach_sensitivity) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        TeachSensitivity teach_sensitivity_res;

        Status status = stub_->SetTeachSensitivity(&context, teach_sensitivity,
                                                   &teach_sensitivity_res);

        return teach_sensitivity_res;
    }

    // Set the xArm position
    Position SetPosition(const Position &position) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        Position position_res;

        Status status = stub_->SetPosition(&context, position, &position_res);

        return position_res;
    }

    // Set the xArm servo_angles
    ServoAngles SetServoAngles(const ServoAngles &servo_angles) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        ServoAngles servo_angles_res;

        Status status =
            stub_->SetServoAngles(&context, servo_angles, &servo_angles_res);

        return servo_angles_res;
    }

    // The motion calculates the trajectory of the space circle according to the
    // three-point coordinates.
    MoveCircleMsg MoveCircle(const MoveCircleMsg &move_circle) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        MoveCircleMsg move_circle_res;

        Status status =
            stub_->MoveCircle(&context, move_circle, &move_circle_res);

        return move_circle_res;
    }

    // Emergency stop
    void EmergencyStop() {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        Empty empty;

        Status status = stub_->EmergencyStop(&context, empty, &empty);

        return;
    }

    // Reset
    void Reset(const ResetMsg &reset) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        Empty empty;

        Status status = stub_->Reset(&context, reset, &empty);

        return;
    }

    // Get inverse kinematics
    ServoAngles GetInverseKinematics(const Position &position) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        ServoAngles servo_angles_res;

        Status status =
            stub_->GetInverseKinematics(&context, position, &servo_angles_res);

        return servo_angles_res;
    }

    // Get forward kinematics
    Position GetForwardKinematics(const ServoAngles &servo_angles) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        Position position_res;

        Status status =
            stub_->GetForwardKinematics(&context, servo_angles, &position_res);

        return position_res;
    }

    // Set the simulation robot
    SimulationRobot SetSimulationRobot(const SimulationRobot &simulation_robot) {
        // Context for the client. It could be used to convey extra information
        // to the server and/or tweak certain RPC behaviors.
        ClientContext context;
        // Container for the data we expect from the server.
        SimulationRobot simulation_robot_res;

        Status status = stub_->SetSimulationRobot(&context, simulation_robot, &simulation_robot_res);

        return simulation_robot_res;
    }


   private:
    // Out of the passed in Channel comes the stub, stored here, our view of the
    // server's exposed services.
    std::unique_ptr<XAPI::Stub> stub_;
};

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
    // Logger configurations
    el::Configurations loggerConf;
    loggerConf.setToDefault();

    // Formatting
    loggerConf.set(el::Level::Verbose, el::ConfigurationType::Format,
                   "%datetime [%level-%vlevel] %msg");
    el::Loggers::reconfigureLogger("default", loggerConf);

    // CLI11 xarm-commander app
    // ClientApp app {
    CLI::App app{
        "xarm-commander: a CLI tool for controlling the UFACTORY xArm using "
        "the gRPC service."};

    // print defaults on help
    app.option_defaults()->always_capture_default();

    // ===== FLAGS =====
    int verbose_level = 0;  // int for supporting multiple flags
    app.add_flag_function(
        "-v, --verbose",
        [&](int verbose_level) {
            el::Loggers::setVerboseLevel(verbose_level);
            VLOG(constants::kLogDebug)
                << "Verbose level: " << verbose_level << std::endl;
        },
        "Verbose mode. To print debugging messages about the progress. "
        "Multiple -v flags increase the verbosity. The maximum is 3.");

    // ===== OPTIONS =====
    std::string server_ip = "127.0.0.1";
    app.add_option("-i, --ip", server_ip, "IP address of the gRPC server");
    std::string server_port = "50051";
    app.add_option("-p, --port", server_port, "port of the gRPC server");

    // ===== SUBCOMMANDS =====
    app.require_subcommand(1);  // set max number of subcommands to 1

    // ----- Connection specific -----
#pragma region disconnect
    // Subcommand: disconnect

    auto *disconnect = app.add_subcommand("disconnect", "disconnect from xArm");
    disconnect->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        client.Disconnect();
        std::cout << "Disconnected" << std::endl;
    });

#pragma endregion disconnect

#pragma region initialize
    // Subcommand: initialize
    auto *initialize = app.add_subcommand("initialize", "initialize XArmAPI");
    std::string xarm_ip = "192.168.0.2";
    initialize->add_option("-x, --xarm_ip", xarm_ip,
                           "ip-address of xArm control box");
    initialize->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        client.Initialize(xarm_ip);  // The actual RPC call!
        std::cout << "Initialized" << std::endl;
    });
#pragma endregion initialize

    // ----- Read methods -----
#pragma region get_collision_sensitivity
    // Subcommand: get_collision_sensitivity
    auto *get_collision_sensitivity =
        app.add_subcommand("get_collision_sensitivity",
                           "send a get_collision_sensitivity command");
    get_collision_sensitivity->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        CollisionSensitivity collision_sensitivity;
        collision_sensitivity =
            client.GetCollisionSensitivity();  // The actual RPC call!
        std::cout << "Collision sensitivity: "
                  << collision_sensitivity.collision_sensitivity() << std::endl;
        std::cout << "Response code: " << collision_sensitivity.status_code()
                  << std::endl;
    });
#pragma endregion get_collision_sensitivity

#pragma region get_teach_sensitivity
    // Subcommand: get_teach_sensitivity
    auto *get_teach_sensitivity = app.add_subcommand(
        "get_teach_sensitivity", "send a get_teach_sensitivity command");
    get_teach_sensitivity->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        TeachSensitivity teach_sensitivity;
        teach_sensitivity =
            client.GetTeachSensitivity();  // The actual RPC call!
        std::cout << "Teach sensitivity: "
                  << teach_sensitivity.teach_sensitivity() << std::endl;
        std::cout << "Response code: " << teach_sensitivity.status_code()
                  << std::endl;
    });
#pragma endregion get_teach_sensitivity

#pragma region get_simulation_robot
    // Subcommand: get_simulation_robot
    auto *get_simulation_robot =
        app.add_subcommand("get_simulation_robot",
                           "send a get_simulation_robot command");
    get_simulation_robot->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        SimulationRobot simulation_robot;
        simulation_robot =
            client.GetSimulationRobot();  // The actual RPC call!
        std::cout << "Simulation mode: "
                  << simulation_robot.on() << std::endl;
        std::cout << "Response code: " << simulation_robot.status_code()
                  << std::endl;
    });
#pragma endregion get_simulation_robot

#pragma region get_version
    // Subcommand: get_version
    auto *get_version =
        app.add_subcommand("get_version", "send a get_version command");
    get_version->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Version version;
        version = client.GetVersion();  // The actual RPC call!
        std::cout << "Version: " << version.version() << std::endl;
        std::cout << "Response code: " << version.status_code() << std::endl;
    });
#pragma endregion get_version

#pragma region get_state
    // Subcommand: get_state
    auto *get_state =
        app.add_subcommand("get_state", "send a get_state commmand");
    get_state->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        State state;
        state = client.GetState();  // The actual RPC call!
        std::cout << "State: " << state.state() << std::endl;
        std::cout << "Response code: " << state.status_code() << std::endl;
    });
#pragma endregion get_state

#pragma region get_cmdnum
    // Subcommand: get_cmdnum
    auto *get_cmdnum =
        app.add_subcommand("get_cmdnum", "send a get_cmdnum commmand");
    get_cmdnum->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Cmdnum cmdnum;
        cmdnum = client.GetCmdnum();  // The actual RPC call!
        std::cout << "Cmdnum: " << cmdnum.cmdnum() << std::endl;
        std::cout << "Response code: " << cmdnum.status_code() << std::endl;
    });
#pragma endregion get_cmdnum

#pragma region get_servo_angles
    // Subcommand: get_servo_angles
    auto *get_servo_angles = app.add_subcommand(
        "get_servo_angles", "send a get_servo_angles command");
    get_servo_angles->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        ServoAngles servo_angles;
        servo_angles = client.GetServoAngles();  // The actual RPC call!
        std::cout << "ServoAngles: \n"
                  << "    \"servo_1\": " << servo_angles.servo_1() << "\n"
                  << "    \"servo_2\": " << servo_angles.servo_2() << "\n"
                  << "    \"servo_3\": " << servo_angles.servo_3() << "\n"
                  << "    \"servo_4\": " << servo_angles.servo_4() << "\n"
                  << "    \"servo_5\": " << servo_angles.servo_5() << "\n"
                  << "    \"servo_6\": " << servo_angles.servo_6() << "\n"
                  << "    \"servo_7\": " << servo_angles.servo_7() << std::endl;
        std::cout << "Response code: " << servo_angles.status_code()
                  << std::endl;
    });
#pragma endregion get_servo_angles

#pragma region get_position
    // Subcommand: get_position
    auto *get_position = app.add_subcommand(
        "get_position",
        "send a get_position command to get the cartesian position");
    get_position->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Position position;
        position = client.GetPosition();  // The actual RPC call!
        std::cout << "Position: \n"
                  << "    \"x\": " << position.x() << "\n"
                  << "    \"y\": " << position.y() << "\n"
                  << "    \"z\": " << position.z() << "\n"
                  << "    \"roll\": " << position.roll() << "\n"
                  << "    \"pitch\": " << position.pitch() << "\n"
                  << "    \"yaw\": " << position.yaw() << std::endl;
        std::cout << "Response code: " << position.status_code() << std::endl;
    });
#pragma endregion get_position

    //----- Write methods -----
#pragma region motion_enable
    // Subcommand: motion_enable
    auto *motion_enable =
        app.add_subcommand("motion_enable", "send a motion_enable command");

    bool enable_flag{true};
    motion_enable->add_flag("-e, --enable, -d{false}, --disable{false}",
                            enable_flag,
                            "enable or disable the xArm (default:"
                            "--enable)");

    int servo_option{constants::kAllServo};
    motion_enable->add_option("-s, --servo", servo_option,
                              "choose servo [1-8] to be enabled/disabled, "
                              "(default: 8 - enable/disable all servo)");

    motion_enable->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        // Request Message
        MotionEnable motion_enable_req;
        motion_enable_req.set_enable(enable_flag);
        motion_enable_req.set_servo_id(servo_option);

        // Response Message
        MotionEnable motion_enable_res;

        motion_enable_res =
            client.SetMotionEnable(motion_enable_req);  // The actual RPC call!
        std::cout << "Response code: " << motion_enable_res.status_code()
                  << std::endl;
    });
#pragma endregion motion_enable

#pragma region set_state
    // Subcommand: set_state
    auto *set_state =
        app.add_subcommand("set_state", "send a set_state command");

    int state_option{0};
    set_state->add_option("-s, --state", state_option,
                          "state, 0: sport, 3: pause, 4: stop");

    set_state->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        State state;
        State state_res;
        state.set_state(state_option);
        state_res = client.SetState(state);  // The actual RPC call!
        std::cout << "Response code: " << state_res.status_code() << std::endl;
    });
#pragma endregion set_state

#pragma region set_mode
    // Subcommand: set_mode
    auto *set_mode = app.add_subcommand("set_mode", "send a set_mode command");

    int mode_option = 0;
    set_mode->add_option(
        "-m", mode_option,
        "mode, 0: position control mode, 1: servo motion mode, 2: joint "
        "teaching "
        "mode, 3: cartesian teaching mode (invalid), 4: simulation mode");

    set_mode->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Mode mode;
        Mode mode_res;
        mode.set_mode(mode_option);
        mode_res = client.SetMode(mode);  // The actual RPC call!
        std::cout << "Response code: " << mode_res.status_code() << std::endl;
    });
#pragma endregion set_mode

#pragma region set_collision_sensitivity
    // Subcommand: set_collision_sensitivity
    auto *set_collision_sensitivity =
        app.add_subcommand("set_collision_sensitivity",
                           "send a set_collision_sensitivity command");

    int collision_sensitivity_option = 0;
    set_collision_sensitivity->add_option("-m", collision_sensitivity_option,
                                          "collision sensitivity value, 0~5");

    set_collision_sensitivity->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        CollisionSensitivity collision_sensitivity;
        CollisionSensitivity collision_sensitivity_res;
        collision_sensitivity.set_collision_sensitivity(
            collision_sensitivity_option);
        collision_sensitivity_res = client.SetCollisionSensitivity(
            collision_sensitivity);  // The actual RPC call!
        std::cout << "Response code: "
                  << collision_sensitivity_res.status_code() << std::endl;
    });
#pragma endregion set_collision_sensitivity

#pragma region set_teach_sensitivity
    // Subcommand: set_teach_sensitivity
    auto *set_teach_sensitivity = app.add_subcommand(
        "set_teach_sensitivity", "send a set_teach_sensitivity command");

    int teach_sensitivity_option = 0;
    set_teach_sensitivity->add_option("-m", teach_sensitivity_option,
                                      "teach sensitivity value, 0~5");

    set_teach_sensitivity->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        TeachSensitivity teach_sensitivity;
        TeachSensitivity teach_sensitivity_res;
        teach_sensitivity.set_teach_sensitivity(teach_sensitivity_option);
        teach_sensitivity_res = client.SetTeachSensitivity(
            teach_sensitivity);  // The actual RPC call!
        std::cout << "Response code: " << teach_sensitivity_res.status_code()
                  << std::endl;
    });
#pragma endregion set_teach_sensitivity

#pragma region set_postion
    // Subcommand: set_position
    auto *set_position = app.add_subcommand(
        "set_position",
        "send a set_position command to set the cartesian position");

    float x_option = constants::kDefaultPosX;
    set_position->add_option("-x", x_option, "x(mm)");
    float y_option = constants::kDefaultPosYaw;
    set_position->add_option("-y", y_option, "y(mm)");
    float z_option = constants::kDefaultPosZ;
    set_position->add_option("-z", z_option, "z(mm)");
    float roll_option = constants::kDefaultPosRoll;
    set_position->add_option("-r, --roll", roll_option, "roll(rad or °)");
    float pitch_option = constants::kDefaultPosPitch;
    set_position->add_option("-p, --pitch", pitch_option, "pitch(rad or °)");
    float yaw_option = constants::kDefaultPosYaw;
    set_position->add_option("-w, --yaw", yaw_option, "yaw(rad or °)");

    bool wait_option = false;
    set_position->add_option("--wait", wait_option,
                             "whether to wait for the arm to complete");

    set_position->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Position position;
        Position position_res;
        position.set_x(x_option);
        position.set_y(y_option);
        position.set_z(z_option);
        position.set_roll(roll_option);
        position.set_pitch(pitch_option);
        position.set_yaw(yaw_option);
        position.set_wait(wait_option);

        position_res = client.SetPosition(position);  // The actual RPC call!
        std::cout << "Response code: " << position_res.status_code()
                  << std::endl;
    });
#pragma endregion set_postion

#pragma region set_servo_angles
    // Subcommand: set_servo_angles
    auto *set_servo_angles = app.add_subcommand(
        "set_servo_angles", "send a set_servo_angles command");

    float servo_1_option = constants::kDefaultAng1;
    set_servo_angles->add_option("-1", servo_1_option, "servo-1(rad or °)");
    float servo_2_option = constants::kDefaultAng2;
    set_servo_angles->add_option("-2", servo_2_option, "servo-2(rad or °)");
    float servo_3_option = constants::kDefaultAng3;
    set_servo_angles->add_option("-3", servo_3_option, "servo-3(rad or °)");
    float servo_4_option = constants::kDefaultAng4;
    set_servo_angles->add_option("-4", servo_4_option, "servo-4(rad or °)");
    float servo_5_option = constants::kDefaultAng5;
    set_servo_angles->add_option("-5", servo_5_option, "servo-5(rad or °)");
    float servo_6_option = constants::kDefaultAng6;
    set_servo_angles->add_option("-6", servo_6_option, "servo-6(rad or °)");
    float servo_7_option = constants::kDefaultAng7;
    set_servo_angles->add_option("-7", servo_7_option, "servo-7(rad or °)");

    wait_option = false;  // TODO(jo-bru): namespace/scope
    set_servo_angles->add_option("--wait", wait_option,
                                 "whether to wait for the arm to complete");

    set_servo_angles->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        ServoAngles servo_angles;
        ServoAngles servo_angles_res;
        servo_angles.set_servo_1(servo_1_option);
        servo_angles.set_servo_2(servo_2_option);
        servo_angles.set_servo_3(servo_3_option);
        servo_angles.set_servo_4(servo_4_option);
        servo_angles.set_servo_5(servo_5_option);
        servo_angles.set_servo_6(servo_6_option);
        servo_angles.set_servo_7(servo_7_option);

        servo_angles.set_wait(wait_option);

        servo_angles_res =
            client.SetServoAngles(servo_angles);  // The actual RPC call!
        std::cout << "Response code: " << servo_angles_res.status_code()
                  << std::endl;
    });
#pragma endregion set_servo_angles

#pragma region move_circle
    // Subcommand: move_circle
    auto *move_circle =
        app.add_subcommand("move_circle", "send a move_circle command");

    float x1_option = constants::kDefaultPosX;
    move_circle->add_option("--x1", x1_option, "x1(mm)");
    float y1_option = constants::kDefaultPosYaw;
    move_circle->add_option("--y1", y1_option, "y1(mm)");
    float z1_option = constants::kDefaultPosZ;
    move_circle->add_option("--z1", z1_option, "z1(mm)");
    float roll1_option = constants::kDefaultPosRoll;
    move_circle->add_option("--r1, --roll1", roll1_option, "roll1(rad or °)");
    float pitch1_option = constants::kDefaultPosPitch;
    move_circle->add_option("--p1, --pitch1", pitch1_option,
                            "pitch1(rad or °)");
    float yaw1_option = constants::kDefaultPosYaw;
    move_circle->add_option("--w1, --yaw1", yaw1_option, "yaw1(rad or °)");

    float x2_option = constants::kDefaultPosX;
    move_circle->add_option("--x2", x2_option, "x2(mm)");
    float y2_option = constants::kDefaultPosYaw;
    move_circle->add_option("--y2", y2_option, "y2(mm)");
    float z2_option = constants::kDefaultPosZ;
    move_circle->add_option("--z2", z2_option, "z2(mm)");
    float roll2_option = constants::kDefaultPosRoll;
    move_circle->add_option("--r2, --roll2", roll2_option, "roll2(rad or °)");
    float pitch2_option = constants::kDefaultPosPitch;
    move_circle->add_option("--p2, --pitch2", pitch2_option,
                            "pitch2(rad or °)");
    float yaw2_option = constants::kDefaultPosYaw;
    move_circle->add_option("--w2, --yaw2", yaw2_option, "yaw2(rad or °)");

    float percent_option = 0;
    move_circle->add_option(
        "--percent", percent_option,
        "the percentage of arc length and circumference of the movement");
    float speed_option = 0;
    move_circle->add_option(
        "--speed", speed_option,
        "move speed (mm/s, rad/s), default(0) is the last_used_tcp_speed");
    float acc_option = 0;
    move_circle->add_option("--acc", acc_option,
                            "move acceleration (mm/s^2, rad/s^2), default(0) "
                            "is the last_used_tcp_acc");

    wait_option = false;
    move_circle->add_option("--wait", wait_option,
                            "whether to wait for the arm to complete");
    float timeout_option = -1;
    move_circle->add_option(
        "--timeout", timeout_option,
        "maximum waiting time(unit: second), -1: no timeout");

    move_circle->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Position *pose_1 = new Position();
        pose_1->set_x(x1_option);
        pose_1->set_y(y1_option);
        pose_1->set_z(z1_option);
        pose_1->set_roll(roll1_option);
        pose_1->set_pitch(pitch1_option);
        pose_1->set_yaw(yaw1_option);

        Position *pose_2 = new Position();
        pose_2->set_x(x2_option);
        pose_2->set_y(y2_option);
        pose_2->set_z(z2_option);
        pose_2->set_roll(roll2_option);
        pose_2->set_pitch(pitch2_option);
        pose_2->set_yaw(yaw2_option);

        MoveCircleMsg move_circle;
        move_circle.set_allocated_pose_1(pose_1);
        move_circle.set_allocated_pose_2(pose_2);

        move_circle.set_percent(percent_option);
        move_circle.set_speed(speed_option);
        move_circle.set_acc(acc_option);
        move_circle.set_wait(wait_option);
        move_circle.set_timeout(timeout_option);

        MoveCircleMsg move_circle_res;

        move_circle_res =
            client.MoveCircle(move_circle);  // The actual RPC call!
        std::cout << "Response code: " << move_circle_res.status_code()
                  << std::endl;

        // TODO: do we need to delete the pointers?
        // delete pose_1;
        // delete pose_2;
    });
#pragma endregion set_move_circle

#pragma region emergency_stop
    // Subcommand: emergency_stop
    auto *emergency_stop =
        app.add_subcommand("emergency_stop", "send an emergency_stop command");

    emergency_stop->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        client.EmergencyStop();  // The actual RPC call!
        std::cout << "Stopped" << std::endl;
    });
#pragma endregion emergency_stop

#pragma region reset
    // Subcommand: reset
    auto *reset = app.add_subcommand("reset", "send a reset command");

    wait_option = false;
    set_teach_sensitivity->add_option(
        "--wait", wait_option, "whether to wait for the arm to complete");

    timeout_option = -1;
    set_teach_sensitivity->add_option(
        "--timeout", timeout_option,
        "maximum waiting time(unit: second), -1: no timeout");

    reset->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        ResetMsg reset;
        reset.set_wait(wait_option);
        reset.set_timeout(timeout_option);
        client.Reset(reset);  // The actual RPC call!
        std::cout << "Reset sent." << std::endl;
    });
#pragma endregion reset

#pragma region get_inverse_kinematics
    // Subcommand: get_inverse_kinematics
    auto *get_inverse_kinematics = app.add_subcommand(
        "get_inverse_kinematics", "send a get_inverse_kinematics command");

    x_option = constants::kDefaultPosX;
    get_inverse_kinematics->add_option("-x", x_option, "x(mm)");
    y_option = constants::kDefaultPosYaw;
    get_inverse_kinematics->add_option("-y", y_option, "y(mm)");
    z_option = constants::kDefaultPosZ;
    get_inverse_kinematics->add_option("-z", z_option, "z(mm)");
    roll_option = constants::kDefaultPosRoll;
    get_inverse_kinematics->add_option("-r, --roll", roll_option,
                                       "roll(rad or °)");
    pitch_option = constants::kDefaultPosPitch;
    get_inverse_kinematics->add_option("-p, --pitch", pitch_option,
                                       "pitch(rad or °)");
    yaw_option = constants::kDefaultPosYaw;
    get_inverse_kinematics->add_option("-w, --yaw", yaw_option,
                                       "yaw(rad or °)");

    get_inverse_kinematics->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        Position position;
        position.set_x(x_option);
        position.set_y(y_option);
        position.set_z(z_option);
        position.set_roll(roll_option);
        position.set_pitch(pitch_option);
        position.set_yaw(yaw_option);
        position.set_wait(wait_option);

        ServoAngles servo_angles;

        servo_angles =
            client.GetInverseKinematics(position);  // The actual RPC call!
        std::cout << "ServoAngles: \n"
                  << "    \"servo_1\": " << servo_angles.servo_1() << "\n"
                  << "    \"servo_2\": " << servo_angles.servo_2() << "\n"
                  << "    \"servo_3\": " << servo_angles.servo_3() << "\n"
                  << "    \"servo_4\": " << servo_angles.servo_4() << "\n"
                  << "    \"servo_5\": " << servo_angles.servo_5() << "\n"
                  << "    \"servo_6\": " << servo_angles.servo_6() << "\n"
                  << "    \"servo_7\": " << servo_angles.servo_7() << std::endl;
        std::cout << "Response code: " << servo_angles.status_code()
                  << std::endl;
    });
#pragma endregion get_inverse_kinematics

#pragma region get_forward_kinematics
    // Subcommand: get_forward_kinematics
    auto *get_forward_kinematics = app.add_subcommand(
        "get_forward_kinematics", "send a get_forward_kinematics command");

    servo_1_option = constants::kDefaultAng1;
    get_forward_kinematics->add_option("-1", servo_1_option,
                                       "servo-1(rad or °)");
    servo_2_option = constants::kDefaultAng2;
    get_forward_kinematics->add_option("-2", servo_2_option,
                                       "servo-2(rad or °)");
    servo_3_option = constants::kDefaultAng3;
    get_forward_kinematics->add_option("-3", servo_3_option,
                                       "servo-3(rad or °)");
    servo_4_option = constants::kDefaultAng4;
    get_forward_kinematics->add_option("-4", servo_4_option,
                                       "servo-4(rad or °)");
    servo_5_option = constants::kDefaultAng5;
    get_forward_kinematics->add_option("-5", servo_5_option,
                                       "servo-5(rad or °)");
    servo_6_option = constants::kDefaultAng6;
    get_forward_kinematics->add_option("-6", servo_6_option,
                                       "servo-6(rad or °)");
    servo_7_option = constants::kDefaultAng7;
    get_forward_kinematics->add_option("-7", servo_7_option,
                                       "servo-7(rad or °)");

    get_forward_kinematics->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        ServoAngles servo_angles;
        servo_angles.set_servo_1(servo_1_option);
        servo_angles.set_servo_2(servo_2_option);
        servo_angles.set_servo_3(servo_3_option);
        servo_angles.set_servo_4(servo_4_option);
        servo_angles.set_servo_5(servo_5_option);
        servo_angles.set_servo_6(servo_6_option);
        servo_angles.set_servo_7(servo_7_option);

        Position position;
        position =
            client.GetForwardKinematics(servo_angles);  // The actual RPC call!
        std::cout << "Position: \n"
                  << "    \"x\": " << position.x() << "\n"
                  << "    \"y\": " << position.y() << "\n"
                  << "    \"z\": " << position.z() << "\n"
                  << "    \"roll\": " << position.roll() << "\n"
                  << "    \"pitch\": " << position.pitch() << "\n"
                  << "    \"yaw\": " << position.yaw() << std::endl;
        std::cout << "Response code: " << position.status_code() << std::endl;
    });
#pragma endregion get_forward_kinematics

#pragma region set_simulation_robot
    // Subcommand: set_simulation_robot
    auto *set_simulation_robot =
        app.add_subcommand("set_simulation_robot", "send a set_simulation_robot command");

    bool on_option = true;
    set_simulation_robot->add_option("-o, --on", on_option,
                          "on: enable(true) or not(false)");

    set_simulation_robot->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        SimulationRobot simulation_robot;
        SimulationRobot simulation_robot_res;
        simulation_robot.set_on(on_option);
        simulation_robot_res = client.SetSimulationRobot(simulation_robot);  // The actual RPC call!
        std::cout << "Response code: " << simulation_robot_res.status_code() << std::endl;
    });
#pragma endregion set_simulation_robot

    CLI11_PARSE(app, argc, argv);

    return 0;
}
