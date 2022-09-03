#ifndef __CERBERUS_CERBERUS_H_
#define __CERBERUS_CERBERUS_H_

#include <cerberus/terminal/menu.h>

#include <type_traits>

#include <absl/status/status.h>
namespace cerberus {
    namespace ctor_args {
        struct SPDLOGArgs {
            const size_t thread_queue_size, thread_count;
        };
    } // namespace ctor_args

    class CerberusDaemon {
      public: //methods
        explicit CerberusDaemon(const ctor_args::SPDLOGArgs& spd);
        ~CerberusDaemon();

        template <cerberus::terminal::TerminalMenu::types Type>
        absl::Status start();

      private: //vars
        cerberus::terminal::TerminalMenu _tm{};
        // const size_t _num_log_thrds, _thrd_q_sz{8192};
    };

    template <cerberus::terminal::TerminalMenu::types TerminalType>
    absl::Status CerberusDaemon::start() {
        using namespace cerberus::terminal; //NOLINT
        absl::Status start_status;
        if constexpr (TerminalMenu::types::TUI == TerminalType) {
            _tm.launch(TerminalType);
        } else if constexpr (TerminalMenu::types::CMDLINE == TerminalType) {
        } else { //GUI
        }
        return start_status;
    }

} // namespace cerberus

#endif //__CERBERUS_CERBERUS_H_
