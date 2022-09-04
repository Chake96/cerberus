#ifndef __CERBERUS_TERMINAL_BASE_MENU_H_
#define __CERBERUS_TERMINAL_BASE_MENU_H_

#include <exception>
#include <string_view>

#include <spdlog/async.h>
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/systemd_sink.h>
#include <spdlog/spdlog.h>

namespace cerberus::terminal {

    class BaseMenu {
      public:
        BaseMenu() {
            try {
                spdlog::register_logger(_file_logger);
            } catch (const spdlog::spdlog_ex& e) {
                //Already Registered
            } catch (const std::exception& e) {
                _file_logger->error(e.what());
            } catch (...) {
                _file_logger->error("Ran into an Unknown Error Registering Base_menu's File Logger");
            }
            try {
                spdlog::register_logger(_systemd_logger);
            } catch (const spdlog::spdlog_ex& e) {

            } catch (const std::exception& e) {
                _systemd_logger->error(e.what());
            } catch (...) {
                ;
                _systemd_logger->error("Ran into an Unknown Error Registering Base_menu's Systemd Logger");
            }
        }
        virtual ~BaseMenu() = default;
        virtual void start() = 0;

      protected:
        //NOLINTBEGIN(readability-identifier-naming)
        inline static const std::string _kfile_logger_name{"menu_file_logger"}, _kconsole_logger_name{"menu_systemd_logger"};
        inline static auto _file_logger = spdlog::basic_logger_mt<spdlog::async_factory>(_kfile_logger_name, "logs/menu_logs.txt");

        inline static auto _console_sink = std::make_shared<spdlog::sinks::systemd_sink_mt>();
        inline static auto _systemd_logger = std::make_shared<spdlog::logger>(_kconsole_logger_name, _console_sink);

        //NOLINTEND(readability-identifier-naming)
    };

} // namespace cerberus::terminal

#endif //__CERBERUS_TERMINAL_BASE_MENU_H_
