//stl
#include <csignal>
#include <iostream>
#include <vector>

//boost
#include <boost/program_options.hpp>

//ftxui
#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>

//cerberus
// #include <cerberus.h>
#include <CLI/CLI.hpp>
#include <cerberus/terminal/menu.h>
#include <spdlog/async.h>
#include <spdlog/spdlog.h>

volatile bool running = true;
void signalHandler([[maybe_unused]] int signal) {
    running = false;
}

namespace ctm = cerberus::terminal;
int main(int argc, const char* argv[]) {

    //setup SPD Log
    static size_t thrd_q_sz{8192};
    static size_t num_log_thrds{1};
    spdlog::init_thread_pool(thrd_q_sz, num_log_thrds); //TODO benchmark this initalizer with various parameters
    // spdlog::set_pattern("[source %s] [function %!] [line %#] %v"); TODO: figure out how to get fine grain file detail into log

    ctm::TerminalMenu tm{};

    static const auto kLaunchTui = [&tm]([[maybe_unused]] size_t call_count) {
        tm.launch(ctm::TerminalMenu::types::TUI);
    };

    CLI::App cmd_line{"Cerberus Daemon"};
    cmd_line.add_flag("--tui", kLaunchTui, "use the TUI Commandline Control Interface");
    cmd_line.add_flag("--headless, --nohead", "launch the daemon in headless server mode, use ./cerberus -h to learn more");
    CLI11_PARSE(cmd_line, argc, argv);

    //cleanup logger globally
    spdlog::shutdown();
    return EXIT_SUCCESS;
}
