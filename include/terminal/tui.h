#ifndef __CERBERUS_TERMINAL_TUI_H_
#define __CERBERUS_TERMINAL_TUI_H_

#include <terminal/base_menu.h>

#include <string>
#include <vector>
#include <chrono>
#include <future>

#include <absl/strings/str_format.h>

#include <ftxui/component/captured_mouse.hpp>  // for ftxui
#include <ftxui/component/component.hpp>       // for CatchEvent, Renderer
#include <ftxui/component/event.hpp>           // for Event
#include <ftxui/component/mouse.hpp>  // for Mouse, Mouse::Left, Mouse::Middle, Mouse::None, Mouse::Pressed, Mouse::Released, Mouse::Right, Mouse::WheelDown, Mouse::WheelUp
#include <ftxui/component/screen_interactive.hpp>  // for ScreenInteractive
#include <ftxui/dom/elements.hpp>  // for text, vbox, window, Element, Elements
#include <ftxui/dom/canvas.hpp>


namespace cerberus::terminal{

class TUITerminal : public BaseMenu {

    public:

        TUITerminal() = default;
        ~TUITerminal(){
            _run.set_value();
        }

        void start(){
            using namespace ftxui;
            _screen.Loop(_main_menu);
        }

    private://vars
        ftxui::ScreenInteractive _screen = ftxui::ScreenInteractive::TerminalOutput();;
        int_fast8_t _top_menu_select{0};

        //for refresh thread
        std::promise<void> _run;


        ftxui::Component _main_menu = ftxui::Container::Vertical({
            ftxui::Button("Quit", _screen.ExitLoopClosure()),
            ftxui::Button("1. Keyboard Controls", [this]{
                _submenu("keyboard");
            }),
        });

    private://funcs
        void _submenu(std::string path){
            using namespace ftxui;
            std::vector<Event> inputs{};

            static auto stringify = [](Event event)->std::string{
                if(event.is_character()){
                    return event.character();
                }
                return "";
            };

            auto screen = ScreenInteractive::TerminalOutput();


            auto renderer = Renderer([&,this]{
                auto c = Canvas(100,100);
                c.DrawText(0,0, "Press Q to go Back");
                Elements children;
                for (size_t i = std::max(0, (int)inputs.size() - 20); i < inputs.size(); ++i){
                    auto str_input = stringify(inputs[i]);
                    if(!str_input.empty()){
                        if(str_input.size() == 1){
                            switch(str_input.front()){
                                case 'Q':[[fallthrough]];
                                case 'q':{
                                    c.DrawText(0,10,"quit!");
                                    screen.Clear();
                                    screen.ExitLoopClosure()();
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                    }
                }
                return 
                    vbox({
                        text("Press Q to go back"),
                        ftxui::separator(),
                        canvas(std::move(c))
                });
            });

            renderer |= CatchEvent([&inputs, this](Event event){
                inputs.push_back(event);
                return true;
            });
            screen.Loop(renderer);
        }


};

}//cerberus::terminal
#endif //__CERBERUS_TERMINAL_TUI_H_