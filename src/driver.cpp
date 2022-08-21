//std + c
#include <iostream>
#include <vector>
#include <signal.h> //TODO: remove with signal handling

//boost
#include <boost/program_options.hpp>
 
//ftxui
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/component/component.hpp>

//cerberus
// #include <cerberus.h>
#include <terminal/menu.h>

namespace po = boost::program_options;

volatile bool running = true;
void signalHandler(int signal)
{
	if (signal == SIGINT
	 || signal == SIGTERM
	 || signal == SIGQUIT)
	{
		running = false;
	}
}

namespace ctm = cerberus::terminal;
int main(int argc, const char* argv[]) {

    //TODO: remove linux based signal handling
    signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);
	signal(SIGQUIT, signalHandler);

    po::options_description descript("Allowed Options");
    descript.add_options()("help", "show the help message")
                            ("tui", "use the Commandline Control Interface")
                            ("TODO Kinect Options", /**/ "TODO");
    po::variables_map inputs;
    po::store(po::parse_command_line(argc, argv, descript), inputs);
    po::notify(inputs);

    bool use_tui{false};

    if(inputs.count("help")){
        std::cout << descript << '\n';
        return 1;
    }

    ctm::TerminalMenu tm{};

    if(inputs.count("tui")){
        tm.launch(ctm::TerminalMenu::types::TUI);
    }



    // if(inputs.count){ whatever options
    // }
            //check if user requested cmdline mode
    return EXIT_SUCCESS;
}
