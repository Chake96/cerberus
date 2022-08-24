#ifndef __CERBERUS_TERMINAL_TUI_H_
#define __CERBERUS_TERMINAL_TUI_H_



#include "boost/lockfree/policies.hpp"
#include <ftxui/dom/deprecated.hpp>
#include <ftxui/screen/color.hpp>
#include <terminal/base_menu.h>
#include <kinect.h>

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
        kinect::Kinect _kin = kinect::Kinect(0, kinect::Kinect::eModels::V1);

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
                static auto state_doc = vflow({});
                Event ele_itr;
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
                                    if(auto state = _kin.tilt_state()){
                                        auto accel_str = absl::StrFormat(
                                            "Accelerometer\n\t X: %f, Y: %f, Z: %f\n",
                                            state->accelerometer_x,
                                            state->accelerometer_y,
                                            state->accelerometer_z
                                        );
                                        auto tilt_str = absl::StrFormat(
                                            "Angle (Degrees): %i Status: %i\n",
                                            state->tilt_angle,
                                            state->tilt_status
                                        );
                                        state_doc = vflow({
                                            text("Tilt Motor State") | bgcolor(Color::Black) | color(Color::White) | bold,
                                            text(accel_str) | bgcolor(Color::White) | color(Color::Blue) ,
                                            text(tilt_str)| bgcolor(Color::White) | color(Color::Blue)
                                        });
                                    }else{
                                        //TODO: log lack of state
                                        static uint16_t count{0};
                                        state_doc = hflow(paragraph(absl::StrFormat("%i #: no state to report for Kinect %i", count++, _kin.kID)));
                                    }
                                    break;
                                }
                                case 'S':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 's':{
                                    if(auto ts = _kin.tilt_state()){
                                        if(ts->tilt_angle > kinect::Kinect::TiltProperties.min){
                                            int current_tilt = ceil(ts->tilt_angle);
                                            current_tilt -= (fast_rate) ? kinect::Kinect::TiltProperties.fast_deg : kinect::Kinect::TiltProperties.slow_deg;
                                            _kin.set_tilt(current_tilt);
                                        }
                                    }else{
                                        _kin.set_tilt(kinect::Kinect::TiltProperties.slow_deg);
                                    }
                                    break;
                                }
                                case 'W':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 'w':{
                                    if(auto ts = _kin.tilt_state()){
                                        if(ts->tilt_angle < kinect::Kinect::TiltProperties.max){
                                            int current_tilt = ceil(ts->tilt_angle);
                                            current_tilt += (fast_rate) ? kinect::Kinect::TiltProperties.fast_deg : kinect::Kinect::TiltProperties.slow_deg;
                                            _kin.set_tilt(current_tilt);
                                        }
                                    }else{
                                        _kin.set_tilt(kinect::Kinect::TiltProperties.slow_deg);
                                    }
                                }break;
                                case 'C':
                                case 'c':{
                                    _kin.set_color((_kin.color() == kinect::units::eLEDColors::GREEN) ? (kinect::units::eLEDColors::RED) : kinect::units::eLEDColors::GREEN);
                                    break;
                                }
                                case 'R':[[fallthrough]];
                                case 'r':{
                                    _kin.set_tilt(0);
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
                        window(text(""), state_doc),
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