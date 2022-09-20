#ifndef __CERBERUS_TERMINAL_TUI_WEBCAM_H_
#define __CERBERUS_TERMINAL_TUI_WEBCAM_H_

#include <cerberus/cameras/manager.h>
#include <cerberus/terminal/base_menu.h>
#include <cerberus/utilities/thread_pool.h>

#include <chrono>
#include <cstdio>
#include <string_view>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <boost/lockfree/spsc_queue.hpp>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include <oneapi/tbb/blocked_range2d.h>
#include <oneapi/tbb/parallel_for.h>

namespace cerberus::terminal {
    namespace tui::paths::webcam {
        static constexpr std::string_view webcams{"webcams"};
        static constexpr std::string_view goto_cams{"gotocam"};

    } // namespace tui::paths::webcam

    class TuiWebcamTerminal : public BaseMenu {
      public: // vars
        static constexpr float kcascade_image_scale{1.5F};

        void start() override { _screen.Loop(_main_menu); }

      public: // methods
        explicit TuiWebcamTerminal(utilities::threads::ThreadPool& tp) : _cams(tp) {
            std::jthread([&, stop_flag = _stop_flags, this] {
                using hrc = std::chrono::high_resolution_clock;
                using namespace std::chrono_literals;
                auto now = hrc::now();
                auto last_tp = now;
                bool added_to_cam{false};
                while (stop_flag.wait_until(last_tp + 5s) != std::future_status::ready) {
                    if (!added_to_cam) {
                        if (auto cam = _cams.get(0)) {
                            added_to_cam = true;
                            cam->register_cv_mat_signal([this](const auto& Mat) {
                                _file_logger->trace("Got new RGB frame");
                                _rgb_stream.push(Mat);
                            });
                        }
                    }
                }
            }).detach();
        }

        ~TuiWebcamTerminal() { _stop_flag_v.set_value(); } // NOLINT(modernize-use-override)

      private: // vars
        cameras::CameraManager _cams;

        // tui
        ftxui::ScreenInteractive _screen = ftxui::ScreenInteractive::TerminalOutput();

        ftxui::Component _renderer;

        std::promise<void> _stop_flag_v;
        std::shared_future<void> _stop_flags = _stop_flag_v.get_future();

        boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<256>> _rgb_stream{}, _depth_stream{};

        struct LockedEvent {
            std::mutex mtx;
            ftxui::Event event;
            std::atomic_bool set{false};
        } _last_event;

        ftxui::Component _main_menu = ftxui::Container::Vertical({
            ftxui::Button("Back", _screen.ExitLoopClosure()),
            ftxui::Button("Streams", [this] { _submenu(tui::paths::webcam::goto_cams); }),
            ftxui::Button("webcams", [this] { _submenu(tui::paths::webcam::webcams); }),
            ftxui::Button(
                "HOG Pedestrian Detection",
                [this] {
                    std::jthread([this] {
                        using namespace std::chrono;          // NOLINT
                        using namespace std::chrono_literals; // NOLINT
                        using namespace cv;                   // NOLINT

                        thread_local Mat rgb(Size(1080, 1920), CV_8UC3, Scalar(0));
                        static thread_local cv::HOGDescriptor hog_d(
                            Size(48, 96),
                            Size(16, 16),
                            Size(8, 8),
                            Size(8, 8),
                            9,
                            1,
                            -1,
                            HOGDescriptor::L2Hys,
                            0.2,
                            true,
                            cv::HOGDescriptor::DEFAULT_NLEVELS
                        );
                        //   hog_d.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
                        hog_d.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
                        thread_local std::vector<cv::Rect> people;
                        thread_local std::vector<double> confidences{};

                        thread_local static auto run_stream = [&] {
                            namedWindow("Pedestrian Tracking");
                            int key_interrupt = pollKey();
                            // Detect faces
                            double detection_threshold{1.0};
                            while (key_interrupt < 0) {
                                if (_rgb_stream.pop(rgb)) {
                                    hog_d.detectMultiScale(
                                        rgb, people, confidences, detection_threshold, Size(8, 8), Size(), 1.05, 2, false
                                    );
                                    if (!people.empty()) {
                                        // Apply rectangles to BGR image
                                        auto confidence = confidences.begin();
                                        for (auto person = people.begin(); person != people.end();
                                             std::advance(person, 1), std::advance(confidence, 1)) {

                                            cv::rectangle(rgb, person->tl(), person->br(), cv::Scalar(0, 255, 0), 2);
                                            if (confidence != confidences.end()) {
                                                cv::putText(
                                                    rgb,
                                                    absl::StrFormat("Confidence: %f", *confidence),
                                                    Point(person->x, person->y),
                                                    FONT_HERSHEY_SIMPLEX,
                                                    1.0,
                                                    Scalar(255, 100, 0),
                                                    2
                                                );
                                            }
                                        }
                                        people.clear();
                                    }
                                    cv::putText(
                                        rgb,
                                        absl::StrFormat("DThreshold: %f", detection_threshold),
                                        Point(5, 25),
                                        FONT_HERSHEY_SIMPLEX,
                                        0.4,
                                        Scalar(255, 100, 0),
                                        2
                                    );
                                    cv::imshow("Pedestrian Tracking", rgb);
                                    // UP: 1113938
                                    // DOWN: 1113940
                                    key_interrupt = pollKey();
                                    if (key_interrupt == 1113938) {
                                        detection_threshold += .1;
                                        key_interrupt = -1;
                                    }
                                    if (key_interrupt == 1113940) {
                                        detection_threshold -= 0.05;
                                        key_interrupt = -1;
                                    }
                                }
                            }
                            destroyAllWindows();
                        };

                        run_stream();
                    });
                }
            ),
        });

      private: // methods
        void _submenu(const std::string_view& path) {
            auto screen = ftxui::ScreenInteractive::FitComponent();
            if (path == tui::paths::webcam::goto_cams) {
                try {
                    _renderer = _cam_renderer(_screen);
                } catch (...) {}
            } else if (path == tui::paths::webcam::webcams) {
                // _renderer = _rgb_stream_renderer(screen);
                std::jthread([this] {
                    using namespace std::chrono;          // NOLINT
                    using namespace std::chrono_literals; // NOLINT
                    using namespace cv;                   // NOLINT
                    using hrc = std::chrono::high_resolution_clock;

                    thread_local static auto run_stream = [&] {
                        namedWindow("rgb");
                        int key_interrupt = pollKey();
                        thread_local Mat rgb(Size(1920, 1080), CV_8UC3, Scalar(0));
                        thread_local Mat cascade_grayscale;
                        thread_local hrc::time_point last_frame_tp = hrc::now();
                        thread_local cv::CascadeClassifier body_cf{"/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml"};
                        static constexpr float kcascade_image_scale{1.5F};
                        const auto swidth = static_cast<int>(static_cast<float>(rgb.size().width) / kcascade_image_scale);
                        const auto sheight = static_cast<int>(static_cast<float>(rgb.size().height) / kcascade_image_scale);
                        std::vector<cv::Rect> bodies;
                        while (key_interrupt < 0) {
                            resize(rgb, cascade_grayscale, cv::Size(swidth, sheight));
                            cvtColor(cascade_grayscale, cascade_grayscale, cv::COLOR_BGR2GRAY);
                            if (!body_cf.empty()) {
                                // Apply rectangles to BGR image
                                for (const auto& body : bodies) {
                                    cv::rectangle(
                                        rgb,
                                        cv::Point(
                                            cvRound(static_cast<float>(body.x) * kcascade_image_scale),
                                            cvRound(static_cast<float>(body.y) * kcascade_image_scale)
                                        ), // Upper left point
                                        cv::Point(
                                            cvRound(
                                                (static_cast<float>(body.x) + static_cast<float>(body.width - 1)) * kcascade_image_scale
                                            ),
                                            cvRound(
                                                (static_cast<float>(body.y) + static_cast<float>(body.height - 1)) * kcascade_image_scale
                                            )
                                        ),                    // Lower right point
                                        cv::Scalar(0, 0, 255) // Red line
                                    );
                                }
                                bodies.clear();
                            }
                            if (_rgb_stream.pop(rgb)) {
                                last_frame_tp = hrc::now();
                            } else {
                                cv::putText(rgb, "no video stream", Point(220, 200), FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 0, 240));
                            }
                            cv::imshow("rgb", rgb);

                            key_interrupt = pollKey();
                        }
                        destroyAllWindows();
                    };

                    run_stream();
                }).join();
                return; // REVISIT: early return
            } else {
                // TODO: log error
                // _file_logger->lo throw std::runtime_error(absl::StrFormat("Failed to initalize Submenu with Path: %s", path));
                _systemd_logger->error(fmt::format("Invalid String path Given to TUI submenu:{}", path));
                _file_logger->error(fmt::format("Invalid String path Given to TUI submenu:{}", path));
            }

            _screen.Loop(_renderer);
        }

        ftxui::Component _cam_renderer(ftxui::ScreenInteractive& screen) {
            using namespace ftxui; // NOLINT
            // A triangle following the mouse, using braille characters.
            Component renderer = Renderer([&] {
                auto c = Canvas(100, 10);
                cv::Mat rgb;
                c.DrawText(
                    0,
                    0,
                    absl::StrFormat("No Stream (%i)", std::chrono::high_resolution_clock::now().time_since_epoch().count()),
                    [](Pixel& p) {
                        p.foreground_color = Color::Red;
                        p.underlined = true;
                        p.bold = true;
                        p.background_color = Color::Black;
                    }
                );
                return canvas(std::move(c));
            });

            renderer |= CatchEvent([&screen](const Event& event) {
                if (event == Event::Character('q') || event == Event::Character('Q')) {
                    screen.ExitLoopClosure();
                    return true;
                }
                return false;
            });

            return renderer;
        }
    };

} // namespace cerberus::terminal

#endif // __CERBERUS_TERMINAL_TUI_WEBCAM_H_