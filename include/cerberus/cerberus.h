#ifndef __CERBERUS_CERBERUS_H_
#define __CERBERUS_CERBERUS_H_

#include "cerberus/cameras/manager.h"
#include <cerberus/terminal/menu.h>
#include <cerberus/utilities/thread_pool.h>

#include <type_traits>

#include <spdlog/async.h>
#include <spdlog/common.h>

#include <absl/status/status.h>

namespace cerberus {
    namespace ctor_args {
        struct SPDLOGArgs {
            const size_t thread_queue_size, thread_count;
            const spdlog::level::level_enum log_level{spdlog::level::err};
        };
    } // namespace ctor_args

    class CerberusDaemon {
      public: // methods
        explicit CerberusDaemon(const ctor_args::SPDLOGArgs& spd);
        ~CerberusDaemon();

        template <cerberus::terminal::TerminalMenu::types Type>
        absl::Status start();

      private: // vars
        static inline auto spd_thrd_pool = spdlog::thread_pool();
        utilities::threads::ThreadPool _thread_pool{{}};
        cerberus::terminal::TerminalMenu _tm;
        // cameras::CameraManager _cam_mgr{_thread_pool};

        // const size_t _num_log_thrds, _thrd_q_sz{8192};
    };

    template <cerberus::terminal::TerminalMenu::types TerminalType>
    absl::Status CerberusDaemon::start() {
        using namespace cerberus::terminal; // NOLINT
        absl::Status start_status;
        if constexpr (TerminalMenu::types::TUI == TerminalType) {
            _tm.launch(TerminalType);
        } else if constexpr (TerminalMenu::types::CMDLINE == TerminalType) {
        } else { // GUI
        }
        return start_status;
    }

} // namespace cerberus

#endif //__CERBERUS_CERBERUS_H_
