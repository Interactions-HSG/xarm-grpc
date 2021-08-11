#include <easyloggingpp/src/easylogging++.h>
#include <xarm/wrapper/xarm_api.h>

#include <CLI11/include/CLI/CLI.hpp>
#include <iostream>
#include <string>

namespace constants {
// Default values (to avoid magic numbers)
constexpr float kDefaultPosX = 300;
constexpr float kDefaultPosY = 0;
constexpr float kDefaultPosZ = 200;
constexpr float kDefaultPosRoll = 180;  // [Deg] by default
constexpr float kDefaultPosPitch = 0;   // [Deg] by default
constexpr float kDefaultPosYaw = 0;     // [Deg] by default

constexpr int kAllServo = 8;

// Logging levels
constexpr int kLogInfo = 1;
constexpr int kLogDebug = 2;
constexpr int kLogEval = 3;
}  // namespace constants

INITIALIZE_EASYLOGGINGPP

auto InitXarm(const std::string &xarm_ip) -> XArmAPI * {
    // TODO(jo-bru): redirect stdout to logger (using pipes?)
    auto *arm = new XArmAPI(xarm_ip);
    return arm;
}

int main(int argc, char **argv) {
    // ===== INIT =====
    int res{1};

    // Logger configurations
    el::Configurations loggerConf;
    loggerConf.setToDefault();
    // Formatting
    loggerConf.set(el::Level::Verbose, el::ConfigurationType::Format,
                   "%datetime [%level-%vlevel] %msg");
    el::Loggers::reconfigureLogger("default", loggerConf);

    // CLI11 xarm-commander app
    CLI::App app{
        "xarm-commander: a command line tool for controlling the xArm7."};

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
    std::string xarm_ip = "localhost";
    app.add_option("-i, --ip", xarm_ip, "ip-address of xArm control box");

    // ===== SUBCOMMANDS =====
    app.require_subcommand(1);  // set max number of subcommands to 1

    // Subcommand: motion_enable
    auto *motion_enable =
        app.add_subcommand("motion_enable", "send a motion_enable command");

    bool enable_flag{true};
    motion_enable->add_flag("-e, --enable, -d{false}, --disable{false}",
                            enable_flag,
                            "enable or disable the xArm (default: --enable)");

    int servo_option{constants::kAllServo};
    motion_enable->add_option("-s, --servo", servo_option,
                              "choose servo [1-8] to be enabled/disabled, "
                              "(default: 8 - enable/disable all servo)");

    motion_enable->callback([&]() {
        auto *arm = InitXarm(xarm_ip);
        res = arm->motion_enable(enable_flag, servo_option);

        VLOG(constants::kLogInfo)
            << "Command: motion_enable " << (enable_flag ? "enable" : "disable")
            << "\n";
        std::cout << "{\"responseCode:\" " << res << "}"
                  << "\n";
    });

    // Subcommand: set_state
    auto *set_state =
        app.add_subcommand("set_state", "send a set_state command");

    int state_option{0};
    set_state->add_option("-s, --state", state_option,
                          "state, 0: sport, 3: pause, 4: stop");

    set_state->callback([&]() {
        auto *arm = InitXarm(xarm_ip);
        res = arm->set_state(state_option);

        VLOG(constants::kLogInfo)
            << "Command: set_state: " << state_option << "\n";
        std::cout << "{\"responseCode:\" " << res << "}"
                  << "\n";
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
        auto *arm = InitXarm(xarm_ip);

        res = arm->set_mode(mode_option);

        VLOG(constants::kLogInfo)
            << "Command: set_mode " << mode_option << "\n";
        std::cout << "{\"responseCode:\" " << res << "}"
                  << "\n";
    });

    // Subcommand: get_version
    auto *get_version =
        app.add_subcommand("get_version", "send a get_version command");
    get_version->callback([&]() {
        auto *arm = InitXarm(xarm_ip);
        res = arm->get_version(arm->version);  // TODO(jo-bru): check if this is
                                               // even needed or arm->version
                                               // holds an updated version

        VLOG(constants::kLogInfo) << "Command: get_version"
                                  << "\n";
        std::cout << "{\"responseCode:\" " << res << ", \n"
                  << "\"responseValue\": "
                  << "{\"version\": " << arm->version << "}"
                  << "}"
                  << "\n";
    });

    // Subcommand: get_state
    auto *get_state =
        app.add_subcommand("get_state", "send a get_state commmand");
    get_state->callback([&]() {
        auto *arm = InitXarm(xarm_ip);

        res = arm->get_state(&arm->state);

        VLOG(constants::kLogInfo) << "Command: get_state "
                                  << "\n";
        std::cout << "{\"responseCode:\" " << res << ", \n"
                  << "\"responseValue\": "
                  << "{\"state\": " << arm->state << "}"
                  << "}"
                  << "\n";
    });

    // Subcommand: get_position
    auto *get_position = app.add_subcommand(
        "get_position",
        "send a get_position command to get the cartesian position");
    get_position->callback([&]() {
        auto *arm = InitXarm(xarm_ip);
        res = arm->get_position(arm->position);

        VLOG(constants::kLogInfo) << "Command: get_position"
                                  << "\n";
        std::cout << "{\"responseCode:\" " << res << ", \n"
                  << "\"responseValue\": "
                  << "{\"position\": {"
                  << "\"x\": " << arm->position[0] << ","
                  << "\"y\": " << arm->position[1] << ","
                  << "\"z\": " << arm->position[2] << ","
                  << "\"roll\": " << arm->position[3] << ","
                  << "\"pitch\": " << arm->position[4] << ","
                  << "\"yaw\": " << arm->position[5] << "}"
                  << "}"
                  << "}"
                  << "\n";
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
        auto *arm = InitXarm(xarm_ip);

        float pose[6];
        pose[0] = x_option;
        pose[1] = y_option;
        pose[2] = z_option;
        pose[3] = roll_option;
        pose[4] = pitch_option;
        pose[5] = yaw_option;

        res = arm->set_position(pose, wait_option);

        VLOG(constants::kLogInfo) << "Command: set_position"
                                  << "\n"
                                  << "Position: [";
        for (float position : pose) {
            VLOG(constants::kLogInfo) << position << " ";
        }
        VLOG(constants::kLogInfo) << "]";
        std::cout << "{\"responseCode:\" " << res << "}"
                  << "\n";
    });

    CLI11_PARSE(app, argc, argv);

    return 0;
}
