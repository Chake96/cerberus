#ifndef __CERBERUS_TERMINAL_TUI_H_
#define __CERBERUS_TERMINAL_TUI_H_

#include "cerberus/utilities/thread_pool.h"
#include <cerberus/cameras/kinect/kinect.h>
#include <cerberus/terminal/base_menu.h>
#include <cerberus/terminal/kinect_tui.h>
#include <cerberus/terminal/webcam_tui.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <absl/strings/str_format.h>

#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>

#include <fmt/core.h>

#include <ftxui/component/captured_mouse.hpp> // for ftxui
#include <ftxui/component/component.hpp>      // for CatchEvent, Renderer
#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp> // for Event
#include <ftxui/component/mouse.hpp> // for Mouse, Mouse::Left, Mouse::Middle, Mouse::None, Mouse::Pressed, Mouse::Released, Mouse::Right, Mouse::WheelDown, Mouse::WheelUp
#include <ftxui/component/screen_interactive.hpp> // for ScreenInteractive
#include <ftxui/dom/canvas.hpp>
#include <ftxui/dom/elements.hpp> // for text, vbox, window, Element, Elements
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/color.hpp>

#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/blocked_range2d.h>
#include <oneapi/tbb/parallel_for.h>

namespace cerberus::terminal {

    namespace tui::paths {
        static constexpr std::string_view kKinect_path{"Kinect"};
        static constexpr std::string_view kUSBWebCam_path{"WebCams"};
    } // namespace tui::paths

    class TUITerminal : public BaseMenu {

      public:
        explicit TUITerminal(cerberus::utilities::threads::ThreadPool& tp) : _tpool(tp) {}
        void start() override { _screen.Loop(_main_menu); }

      private: // vars
        cerberus::utilities::threads::ThreadPool& _tpool;

        std::shared_ptr<cerberus::terminal::BaseMenu> _tui;
        // tui
        ftxui::ScreenInteractive _screen = ftxui::ScreenInteractive::TerminalOutput();

        boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<256>> _rgb_stream{}, _depth_stream{};

        struct LockedEvent {
            std::mutex mtx;
            ftxui::Event event;
            std::atomic_bool set{false};
        } _last_event;

        void _submenu(const std::string_view& path) {
            auto screen = ftxui::ScreenInteractive::FitComponent();
            if (_tui)
                _tui.reset();
            if (path == tui::paths::kKinect_path) {
                _tui = std::dynamic_pointer_cast<BaseMenu>(std::make_shared<cerberus::terminal::TUIKinectTerminal>());
                _tui->start();
            } else if (path == tui::paths::kUSBWebCam_path) {
                try {
                    _tui = std::dynamic_pointer_cast<BaseMenu>(std::make_shared<cerberus::terminal::TuiWebcamTerminal>(_tpool));
                    // _tui = tui;
                    if (_tui) {
                        _tui->start();
                    } else {
                        _file_logger->error("Creating TUITerminal failed\n");
                    }
                } catch (...) {
                    _file_logger->error("Unabled to start TUI Webcam Terminal\n");
                }
            } else {
                // TODO: log error
                // _file_logger->lo throw std::runtime_error(absl::StrFormat("Failed to
                // initalize Submenu with Path: %s", path));
                _systemd_logger->error(fmt::format("Invalid String path Given to TUI submenu:{}", path));
                _file_logger->error(fmt::format("Invalid String path Given to TUI submenu:{}", path));
            }

            _tui.reset();

            _screen.Loop(_main_menu);
        }

        ftxui::Component _main_menu = ftxui::Container::Vertical({
            ftxui::Button("Quit", _screen.ExitLoopClosure(), ftxui::ButtonOption::Animated(ftxui::Color::Red)),
            ftxui::Button("1. Kinect", [this] { _submenu(tui::paths::kKinect_path); }),
            ftxui::Button("2. Webcams", [this] { _submenu(tui::paths::kUSBWebCam_path); }),
        });
    };

} // namespace cerberus::terminal
#endif //__CERBERUS_TERMINAL_TUI_H_