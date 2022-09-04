//stl

#include <csignal>
#include <iostream>
#include <vector>

//cerberus
#include <cerberus/cerberus.h>

#include <CLI/CLI.hpp>

volatile bool running = true;
void signalHandler([[maybe_unused]] int signal) {
    running = false;
}

int main(int argc, const char* argv[]) {
    using TerminalMenuTypes = cerberus::terminal::TerminalMenu::types;

    //TODO benchmark this initalizer with various parameters
    //setup SPD Log
    cerberus::ctor_args::SPDLOGArgs spd_args{.thread_queue_size = 8192, .thread_count = 1};
    bool launch_tui{false}, launch_gui{false}, launch_cmdline{false}; //NOLINT
    CLI::App cmd_line{"Cerberus Daemon"};
    cmd_line.add_flag("--tui", launch_tui, "use the TUI Commandline Control Interface");
    cmd_line.add_flag("--gui", launch_gui, "use the GUI to control cerberus");
    cmd_line.add_flag("--cmdline, --cl", launch_cmdline, "use cerberus and control it with a plain terminal");
    // cmd_line.add_flag("--headless, --nohead", "launch the daemon in headless server mode, use ./cerberus -h to learn more");
    CLI11_PARSE(cmd_line, argc, argv);

    cerberus::CerberusDaemon daemon(spd_args);
    absl::Status daemon_status;
    if (launch_tui) {
        daemon_status = daemon.start<TerminalMenuTypes::TUI>();
    } else if (launch_gui) {
        daemon_status = daemon.start<TerminalMenuTypes::GUI>();
    } else if (launch_cmdline) {
        daemon_status = daemon.start<TerminalMenuTypes::CMDLINE>();
    } else { //headless
    }
    if (!daemon_status.ok()) {
        std::cerr << fmt::format("Failed to Start TUI Terminal. Returned with Value: {}", daemon_status.message());
    }

    //cleanup logger globally
    return EXIT_SUCCESS;
}
