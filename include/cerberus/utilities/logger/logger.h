#ifndef __CERBERUS_UTILITIES_LOGGER_H_
#define __CERBERUS_UTILITIES_LOGGER_H_

#include <memory>
#include <string_view>
#include <unordered_map>

#include <spdlog/logger.h>
#include <spdlog/sinks/sink.h>

#include <absl/container/flat_hash_map.h>

namespace cerberus::utilities::logger {
    template <class SinkType>
    struct LoggerDescription {
        const std::string_view name;
        const std::shared_ptr<SinkType> sink;
    };

    enum class eLogState { ERROR = -1, OK = 0 };

    struct LogStatus {
        eLogState state;
        std::string msg{};
    };

    class LogService {

      public:
        explicit LogService(bool tui_enabled) : _tui_enabled(tui_enabled) {}

        template <class SinkType, typename... SinkArgs>
        std::optional<LogStatus> add_logger(const std::vector<std::string_view>& names, SinkArgs&&... args) {
            LogStatus status;
            for (const auto& name : names) {
                if (!_loggers.contains(name)) {
                    _loggers[name] = std::make_shared(name, std::make_shared<SinkType>(std::forward<SinkArgs>(args)...));
                } else {
                    status.state = eLogState::ERROR;
                    auto append_msg = fmt::format("Logger with name {} Already Registered\n", name);
                    status.msg.append(append_msg);
                    return status;
                }
            }
            return std::make_optional(status);
        }

      private:
        bool _tui_enabled{false};
        absl::flat_hash_map<std::string_view, std::shared_ptr<spdlog::logger>> _loggers{};
    };

} // namespace cerberus::utilities::logger

#endif // __CERBERUS_UTILITIES_LOGGER_H_