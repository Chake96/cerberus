#include <terminal/menu.h>

#include <stddef.h>  
#include <algorithm> 
#include <memory>    
#include <string> 
#include <utility>  
#include <vector>   
#include <functional>
#include <mutex>
	


namespace cerberus::terminal{

    namespace{
        using namespace ftxui;
        using types = TerminalMenu::types;
        using namespace oneapi;
    }

    void TerminalMenu::launch(types launch_type) noexcept{
        try{
            std::call_once(_launch_once, &TerminalMenu::_launch, this, launch_type);
        }catch(...){
            //TODO: log multiple calls to launch terminal
        }        
    }

    void TerminalMenu::_launch(types launch_type) noexcept{
        switch(launch_type){
            case types::CMDLINE:
                if(_launch_cmdline() != TerminalMenu::launch_result::OK){
                    //TODO: log error
                }
                break;
            case types::TUI:
                if(_launch_tui() != TerminalMenu::launch_result::OK){
                    //TODO: log error
                }
                break;
            case types::GUI:
                if(_launch_gui() != TerminalMenu::launch_result::OK){
                    //TODO: log error
                }
                break;
        }
    }

    TerminalMenu::launch_result TerminalMenu::_launch_tui(){
        using namespace ftxui;
        
        launch_result res{launch_result::OK};
        try{
            auto screen = ScreenInteractive::TerminalOutput();
            
            static const std::vector<std::string> entries = {
                "keyboard control",
                "entry 2",
                "entry 3",
            };
            int selected = 0;
            
            MenuOption option;
            option.on_enter = screen.ExitLoopClosure();
            auto menu = Menu(&entries, &selected, &option);
            // screen.Loop(menu);
            // auto screen_loop_fn = std::mem_fn(&ScreenInteractive::Loop);
            // auto binded_fn = std::bind(screen_loop_fn, &screen, menu);
            // _tsk_arena.enqueue(binded_fn);
            screen.Loop(menu);
            
            std::cout << "Selected element = " << selected << std::endl;
        }catch(...){
            res = launch_result::ERROR;
        }
        return res;
    }

    TerminalMenu::launch_result TerminalMenu::_launch_gui(){
        launch_result res{launch_result::OK};
        return res;
    }

    TerminalMenu::launch_result TerminalMenu::_launch_cmdline(){
        launch_result res{launch_result::OK};
        return res;
    }

} //end namespace
