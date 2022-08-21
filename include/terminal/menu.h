
#include <oneapi/tbb/task_arena.h>
 

namespace cerberus::terminal{

    class TerminalMenu{

        public: //vars 
        enum class types{CMDLINE, TUI, GUI};

        public://funcs
          TerminalMenu() = default;
          
          void launch(types launch_type) noexcept;

        

        private: //vars
            enum class launch_result{ERROR, OK};
            // std::future<TerminalMenu::launch_result> _launch_fut;

            //threading
            oneapi::tbb::task_arena _tsk_arena;
            std::once_flag _launch_once;

        private: //functions
            void _launch(types launch_type) noexcept;

            TerminalMenu::launch_result _launch_tui();

            TerminalMenu::launch_result _launch_gui();

            TerminalMenu::launch_result _launch_cmdline();

    };
} //end namespace
