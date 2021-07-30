#include <CLI11.hpp>
#include <xarm/wrapper/xarm_api.h>

#include <string>
#include <iostream>

void xarm_init(XArmAPI *arm)
{
}

int main(int argc, char **argv)
{
    XArmAPI *arm;
    std::string port = "130.82.171.9";
    int res = 1;

    CLI::App app{"xarm-commander: a command line tool for controlling the xArm7."};

    // ===== FLAGS =====
    bool print_mode;
    app.add_flag("-p", print_mode, "Print additional information");

    app.require_subcommand(1); // set max number of subcommands to 1

    // ===== SUBCOMMANDS =====
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

    // Subcommand: motion_enable
    auto motion_enable = app.add_subcommand("motion_enable", "Motion enable");

    bool enable_option = true;
    motion_enable->add_option("-b", enable_option, "Enable or disable"); // TODO: ,true : not working?

    int servo_option = 8;
    motion_enable->add_option("-s", servo_option, "Choose servo (1-8), 8: enable/disable all servo ");

    motion_enable->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->motion_enable(enable_option, servo_option);

        if (print_mode)
            std::cout << "Motion enable - Response: " << res << "\n";
    });

    CLI11_PARSE(app, argc, argv);

    std::cout << "\nThanks for using commander!\n"
              << std::endl;
    return 0;
}
