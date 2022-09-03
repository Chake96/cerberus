#include <cerberus/cerberus.h>
#include <cerberus/terminal/menu.h>

#include <spdlog/async.h>
#include <spdlog/spdlog.h>

namespace cerberus {
    using namespace ctor_args;          //NOLINT
    using namespace cerberus::terminal; //NOLINT

    CerberusDaemon::CerberusDaemon(const SPDLOGArgs& spd) {
        spdlog::init_thread_pool(spd.thread_queue_size, spd.thread_count);
        // spdlog::set_pattern("[source %s] [function %!] [line %#] %v"); TODO: figure out how to get fine grain file detail into log
    }

    CerberusDaemon::~CerberusDaemon() {
        spdlog::shutdown();
    }

} // namespace cerberus