#include "../include/CLI11.hpp"
#include "../lib/xArm-CPLUS-SDK/include/xarm/wrapper/xarm_api.h"

#include <string>

int main(int argc, char **argv)
{

    CLI::App app{"xarm-commander: a command line tool for controlling the xArm7."};

    bool print_mode;
    app.add_flag("-p", print_mode, "Print additional information");

    app.require_subcommand(1); // set max number of subcommands to 1

    auto get_version = app.add_subcommand("get_version", "Get the xArm version");
    get_version->callback([&]() {
        std::string version = "v1.0";
        if (print_mode)
            std::cout << "Get version - Response:";
        std::cout << version;
    });
}
