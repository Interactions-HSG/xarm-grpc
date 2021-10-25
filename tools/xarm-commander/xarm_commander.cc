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
constexpr float kDefaultPosX = 300;
constexpr float kDefaultPosY = 0;
constexpr float kDefaultPosZ = 200;
constexpr float kDefaultPosRoll = 180;  // [Deg] by default
constexpr float kDefaultPosPitch = 0;   // [Deg] by default
constexpr float kDefaultPosYaw = 0;     // [Deg] by default
constexpr int kAllServo = 8;
constexpr int kLogInfo = 1;
constexpr int kLogDebug = 2;
constexpr int kLogEval = 3;
}  // namespace constants

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using xapi::Empty;
using xapi::InitParam;
using xapi::Mode;
using xapi::MotionEnable;
using xapi::Position;
using xapi::State;
using xapi::Version;
using xapi::XAPI;

/*
class ClientApp: public CLI::App {
public:
    // XAPIClient
    XAPIClient client;

    // initialization after parsing and before callback
    // overriding the virtual function
    void pre_callback() override {
        std::string server_ip;
        std::string server_port;

        client = new XAPIClient(
                grpc::CreateChannel(fmt::format("{}:{}", server_ip,
server_port), grpc::InsecureChannelCredentials())
        );
    }
};
*/

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

    // ===== Properties =====
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

    // Get the xArm position
    Position GetPosition() {
        Empty empty;
        Position position;
        ClientContext context;
        Status status = stub_->GetPosition(&context, empty, &position);
        return position;
    }

    // ===== Actions =====

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
    // Subcommand: disconnect
    auto *disconnect = app.add_subcommand("disconnect", "disconnect from xArm");
    disconnect->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        client.Disconnect();
        std::cout << "Disconnected" << std::endl;
    });

    // Subcommand: initialize
    std::string xarm_ip = "192.168.0.2";
    auto *initialize = app.add_subcommand("initialize", "initialize XArmAPI");
    initialize->add_option("-x, --xarm_ip", xarm_ip,
                           "ip-address of xArm control box");
    initialize->callback([&]() {
        XAPIClient client(
            grpc::CreateChannel(fmt::format("{}:{}", server_ip, server_port),
                                grpc::InsecureChannelCredentials()));
        client.Initialize(xarm_ip);  // The actual RPC call!
        std::cout << "Initialized" << std::endl;
    });

    // ----- Properties -----

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

    // ===== Actions =====
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

    CLI11_PARSE(app, argc, argv);

    return 0;
}
