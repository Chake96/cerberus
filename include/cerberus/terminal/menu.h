#ifndef __CERBERUS_TERMINAL_MENU_H_
#define __CERBERUS_TERMINAL_MENU_H_

#include "cerberus/utilities/thread_pool.h"
#include <cerberus/terminal/base_menu.h>
#include <cerberus/terminal/tui.h>

#include <oneapi/tbb/task_arena.h>

namespace cerberus::terminal {

    class TerminalMenu {

      public: // vars
        enum class types { CMDLINE, TUI, GUI };

      public: // funcs
        explicit TerminalMenu(utilities::threads::ThreadPool& tp) : _tpool(tp) {}

        void launch(types launch_type) noexcept;

      private: // vars
        enum class launch_result { ERROR, OK };
        std::unique_ptr<BaseMenu> _menu_inst;

        // threading
        utilities::threads::ThreadPool& _tpool;
        std::once_flag _launch_once;

      private: // functions
        void _launch(types launch_type) noexcept;

        TerminalMenu::launch_result _launch_tui();

        TerminalMenu::launch_result _launch_gui();

        TerminalMenu::launch_result _launch_cmdline();
    };
} // namespace cerberus::terminal

#endif //__CERBERUS_TERMINAL_MENU_H_