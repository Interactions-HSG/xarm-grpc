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
    // Subcommand: get_version TODO: not working correctly
    auto get_version = app.add_subcommand("get_version", "Get the xArm version");
    get_version->callback([&]() {
        arm = new XArmAPI(port);
        res = arm->get_version(arm->version);

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

    CLI11_PARSE(app, argc, argv);

    std::cout << "\nThanks for using commander!\n"
              << std::endl;
    return 0;
}
