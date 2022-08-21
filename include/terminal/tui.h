 
#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"       // for CatchEvent, Renderer
#include "ftxui/component/event.hpp"           // for Event
#include "ftxui/component/mouse.hpp"  // for Mouse, Mouse::Left, Mouse::Middle, Mouse::None, Mouse::Pressed, Mouse::Released, Mouse::Right, Mouse::WheelDown, Mouse::WheelUp
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, vbox, window, Element, Elements

using namespace ftxui;

class TUITerminal{

    private://vars
        Screen

        struct{
            static const control_descript = hbox({
                text("tilt-up: Arrow-Key UP") | border,
                text("tilt-down: Arrow-Key DOWN") | border
            });


        }_keyboard_cntrl;
};