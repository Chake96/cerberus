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
#include <cerberus/terminal/menu.h>

namespace po = boost::program_options;

volatile bool running = true;
void signalHandler([[maybe_unused]] int signal) {
  running = false;
}

namespace ctm = cerberus::terminal;
int main(int argc, const char* argv[]) {

  //TODO: remove linux based signal handling
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGQUIT, signalHandler);

  po::options_description descript("Allowed Options");
  descript.add_options()("help", "show the help message")(
      "tui", "use the Commandline Control Interface")("TODO Kinect Options",
                                                      /**/ "TODO");
  po::variables_map inputs;
  po::store(po::parse_command_line(argc, argv, descript), inputs);
  po::notify(inputs);

  if (inputs.count("help") > 0) {
    std::cout << descript << '\n';
    return 1;
  }

  ctm::TerminalMenu tm{};

  if (inputs.count("tui") > 0) {
    tm.launch(ctm::TerminalMenu::types::TUI);
  }

  // if(inputs.count){ whatever options
  // }
  //check if user requested cmdline mode
  return EXIT_SUCCESS;
}
