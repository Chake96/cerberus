#ifndef __CERBERUS_TERMINAL_TUI_H_
#define __CERBERUS_TERMINAL_TUI_H_



#include "SI/angle.h"
#include "boost/lockfree/policies.hpp"
#include <ftxui/dom/deprecated.hpp>
#include <ftxui/screen/color.hpp>
#include <terminal/base_menu.h>
#include <kinect.h>
#include <kinect_manager.h>

#include <string>
#include <vector>
#include <chrono>
#include <future>

#include <absl/strings/str_format.h>

#include <boost/lockfree/spsc_queue.hpp>

#include <ftxui/component/component_base.hpp>
#include <ftxui/dom/node.hpp>
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

        void start(){
            using namespace ftxui;
            _renderer = Renderer([&]{return _main_menu->Render();});
            _screen.Loop(_main_menu);
        }

    private://vars
        kinect::Kinect _kin = kinect::Kinect(0);

        //tui
        ftxui::ScreenInteractive _screen = ftxui::ScreenInteractive::TerminalOutput();

        ftxui::Component _renderer;


        void _submenu(std::string path){
            using namespace ftxui;
            // std::vector<Event> inputs{};
            boost::lockfree::spsc_queue<Event,boost::lockfree::capacity<10>> inputs{};

            static auto stringify = [](const Event& event)->std::string{
                if(event.is_character()){
                    return event.character();
                }
                return "";
            };

            auto screen = ScreenInteractive::FitComponent();


            _renderer = Renderer([&, this]{
                using namespace SI;
                using namespace SI::literals;


                static auto state_doc = vflow({});
                Event ele_itr;
                double acc_x{}, acc_y{}, acc_z{};
                auto new_state = _kin.get_state();//TODO fix not checking optional
                // new_state.getAccelerometers(&acc_x, &acc_y, &acc_z);
                auto accel_str = absl::StrFormat(
                    "Accelerometer\n\t X: %f, Y: %f, Z: %f\n",
                    acc_x, acc_y, acc_z
                );
                auto tilt_str = absl::StrFormat(
                    "Angle (Degrees): %f\n",
                    new_state->cmded_angle.value()
                );
                if(inputs.pop(ele_itr)){
                    auto str_input = stringify(ele_itr);
                    if(!str_input.empty()){
                        if(str_input.size() == 1){
                            bool fast_rate{false};
                            switch(str_input.front()){
                                case 'Q':[[fallthrough]];
                                case 'q':{
                                    screen.ResetPosition(true);
                                    screen.ExitLoopClosure()();
                                    break;
                                }
                                case 'P':[[fallthrough]];
                                case 'p':{
                                    
                                    break;
                                }
                                case 'S':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 's':{
                                    double dif = (fast_rate) ? kinect::units::TiltProperties::fast_step : kinect::units::TiltProperties::slow_step;
                                    auto new_ang = degree_t<double>{new_state->cmded_angle.value() - dif};
                                    _kin.set_tilt(new_ang);
                                    break;
                                }
                                case 'W':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 'w':{
                                    double dif = (fast_rate) ? kinect::units::TiltProperties::fast_step : kinect::units::TiltProperties::slow_step;
                                    auto new_ang = degree_t<double>{new_state->cmded_angle.value() + dif};
                                    _kin.set_tilt(new_ang);
                                }break;
                                case 'C':
                                case 'c':{
                                    break;
                                }
                                case 'R':[[fallthrough]];
                                case 'r':{
                                    _kin.set_tilt(degree_t<double>{0});
                                }
                                default:
                                    break;
                            }
                        }
                    }
                }
                
                state_doc = vflow({
                    text("Tilt Motor State") | bgcolor(Color::Black) | color(Color::White) | bold,
                    text(accel_str) | bgcolor(Color::White) | color(Color::Blue) ,
                    text(tilt_str)| bgcolor(Color::White) | color(Color::Blue)
                });

                return 
                    vbox({
                        text("Press Q to go back"),
                        ftxui::separator(),
                        window(text("Kinect State"), state_doc),
                    });
            });

            _renderer |= CatchEvent([&inputs](Event event){
                if(event.is_character()){
                    inputs.push(event);
                    return true;
                }
                return false;
            });
            screen.Loop(_renderer);
        }


        ftxui::Component _main_menu = ftxui::Container::Vertical({
            ftxui::Button("Quit", _screen.ExitLoopClosure()),
            ftxui::Button("1. Keyboard Controls", [this]{
                _submenu("keyboard");
            }),
        });

    private://funcs
        

};

}//cerberus::terminal
#endif //__CERBERUS_TERMINAL_TUI_H_