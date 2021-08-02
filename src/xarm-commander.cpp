#include <CLI11.hpp>
#include <xarm/wrapper/xarm_api.h>

#include <string>
#include <iostream>
#include <chrono>

int main(int argc, char **argv)
{
    XArmAPI *arm;
    std::string port = "130.82.171.9";
    int res = 1;

    CLI::App app{"xarm-commander: a command line tool for controlling the xArm7."};

    // print defaults on help
    app.option_defaults()->always_capture_default();

    // ===== FLAGS =====
    bool print_mode = false;
    app.add_flag("-v, --verbose", print_mode, "Verbose mode."); // TODO: multilevel verbose modes + logger

    app.require_subcommand(1); // set max number of subcommands to 1

    // ===== SUBCOMMANDS =====
    // Subcommand: motion_enable
    auto motion_enable = app.add_subcommand("motion_enable", "Motion enable");

    bool enable_option = true;
    motion_enable->add_option("-e, --enable", enable_option, "Enable or disable");

    int servo_option = 8;
    motion_enable->add_option("-s, --servo", servo_option, "Choose servo (1-8), 8: enable/disable all servo ");

    motion_enable->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->motion_enable(enable_option, servo_option);

        if (print_mode)
            std::cout << "Motion enable - Response: " << res << "\n";
    });

    // Subcommand: set_state
    auto set_state = app.add_subcommand("set_state", "Set the xArm state");

    int state_option{0};
    set_state->add_option("-s, --state", state_option, "State, 0: sport, 3: pause, 4: stop");

    set_state->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->set_state(state_option);

        if (print_mode)
            std::cout << "Set state: " << state_option << " - Response: " << res << "\n";
    });

    // Subcommand: set_mode
    auto set_mode = app.add_subcommand("set_mode", "Set the xArm mode");

    int mode_option = 0;
    set_mode->add_option("-m", mode_option, "Mode, 0: position control mode, 1: servo motion mode, 2: joint teaching mode, 3: cartesian teaching mode (invalid), 4: simulation mode");

    set_mode->callback([&]() {
        arm = new XArmAPI(port);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        res = arm->set_mode(mode_option);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

        if (print_mode)
            std::cout << "Set mode: " << mode_option << " - Response: " << res << "\n";
    });

    // Subcommand: get_version
    auto get_version = app.add_subcommand("get_version", "Get the xArm version");
    get_version->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->get_version(arm->version); // TODO: check if this is even needed or arm->version holds an updated version

        if (print_mode)
            std::cout << "Get version - Response: " << res << "\n"
                      << "Version: ";
        std::cout << arm->version;
    });

    // Subcommand: get_state
    auto get_state = app.add_subcommand("get_state", "Get the xArm state");
    get_state->callback([&]() {
        arm = new XArmAPI(port);

        res = arm->get_state(&arm->state);

        if (print_mode)
            std::cout << "Get state - Response: " << res << "\n"
                      << "State: ";
        std::cout << arm->state;
    });

    // Subcommand: get_position
    auto get_position = app.add_subcommand("get_position", "Get the cartesian position");
    get_position->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->get_position(arm->position);

        if (print_mode)
            std::cout << "Get position - Response: " << res << "\n"
                      << "Position: ";
        std::cout << "[ ";
        for (int i = 0; i < 6; i++)
            std::cout << arm->position[i] << " ";
        std::cout << "]";
    });

    // Subcommand: set_position
    auto set_position = app.add_subcommand("set_position", "Set the cartesian position");

    float x_option = 300;
    set_position->add_option("-x", x_option, "x(mm)");
    float y_option = 0;
    set_position->add_option("-y", y_option, "y(mm)");
    float z_option = 200;
    set_position->add_option("-z", z_option, "z(mm)");
    float roll_option = 180;
    set_position->add_option("-r, --roll", roll_option, "roll(rad or °)");
    float pitch_option = 0;
    set_position->add_option("-p, --pitch", pitch_option, "pitch(rad or °)");
    float yaw_option = 0;
    set_position->add_option("-w, --yaw", yaw_option, "yaw(rad or °)");

    bool wait_option = false;
    set_position->add_option("--wait", wait_option, "whether to wait for the arm to complete");

    set_position->callback([&]() {
        arm = new XArmAPI(port);

        float pose[6];
        pose[0] = x_option;
        pose[1] = y_option;
        pose[2] = z_option;
        pose[3] = roll_option;
        pose[4] = pitch_option;
        pose[5] = yaw_option;

        res = arm->set_position(pose, wait_option);

        if (print_mode)
        {
            std::cout << "Set position - Response: " << res << "\n"
                      << "Position: ";
            std::cout << "[ ";
            for (int i = 0; i < 6; i++)
                std::cout << pose[i] << " ";
            std::cout << "]";
        }
    });

    CLI11_PARSE(app, argc, argv);

    std::cout << "\nThanks for using xarm-commander!\n"
              << std::endl;
    return 0;
}
