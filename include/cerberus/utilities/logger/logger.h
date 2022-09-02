#ifndef __CERBERUS_UTILITIES_LOGGER_H_
#define __CERBERUS_UTILITIES_LOGGER_H_

namespace cerberus::utilities::logger {

    class BaseLogger {

      public:
        explicit BaseLogger(bool tui_enabled) : _tui_enabled(tui_enabled) {}

      private:
        bool _tui_enabled{false};
    };

} // namespace cerberus::utilities::logger

#endif // __CERBERUS_UTILITIES_LOGGER_H_