#ifndef __CERBERUS_TERMINAL_TUI_H_
#define __CERBERUS_TERMINAL_TUI_H_

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <SI/angle.h>
#include <absl/strings/str_format.h>
#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <cerberus/kinect.h>
#include <cerberus/kinect_manager.h>
#include <cerberus/terminal/base_menu.h>
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
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace cerberus::terminal {

    namespace tui::paths {
        static constexpr std::string_view keyboard{"Keyboard"};
        static constexpr std::string_view rgb_stream{"RGB Stream"};
    } // namespace tui::paths

    class TUITerminal : public BaseMenu {

      public:
        TUITerminal() = default;

        void start() override { _screen.Loop(_main_menu); }

      private: // vars
        kinect::Kinect<kinect::CVNect> _kin =
            kinect::Kinect<kinect::CVNect>(0, std::bind(std::mem_fn(&TUITerminal::_rgb_stream_cb), this, // NOLINT
                                                        std::placeholders::_1));                         // NOLINT
                                                                                                         //OpenCV
        // cv::CascadeClassifier _face_cf{"/usr/share/opencv4/lbpcascades/lbpcascade_frontalface_improved.xml"}; //TODO: implement faster CV model

        // tui
        ftxui::ScreenInteractive _screen = ftxui::ScreenInteractive::TerminalOutput();

        ftxui::Component _renderer;

        boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<256>> _rgb_stream{};

        struct LockedEvent {
            std::mutex mtx;
            ftxui::Event event;
            std::atomic_bool set{false};
        } _last_event;

        void _submenu(const std::string_view& path) {
            auto screen = ftxui::ScreenInteractive::FitComponent();
            if (path == tui::paths::keyboard) {
                try {
                    _renderer = _keyboard_renderer(screen);
                } catch (...) {}
            } else if (path == tui::paths::rgb_stream) {
                _renderer = _rgb_stream_renderer(screen);
            } else {
                // TODO: log error
                throw std::runtime_error(absl::StrFormat("Failed to initalize Submenu with Path: %s", path));
            }

            screen.Loop(_renderer);
        }

        //CV common variables
        static constexpr float cascade_image_scale{1.5F};

        ftxui::Component _main_menu = ftxui::Container::Vertical({
            ftxui::Button("Quit", _screen.ExitLoopClosure()),
            ftxui::Button("1. Keyboard Controls", [this] { _submenu(tui::paths::keyboard); }),
            ftxui::Button("2. View Terminal RGB Stream", [this] { _submenu(tui::paths::rgb_stream); }),
            ftxui::Button("3. Show Stream",
                          [this] {
                              std::jthread([this] {
                                  using namespace std::chrono;          // NOLINT
                                  using namespace std::chrono_literals; // NOLINT
                                  using namespace cv;                   // NOLINT

                                  thread_local auto start = std::chrono::high_resolution_clock::now(), // NOLINT
                                      now = start;                                                     // NOLINT
                                  thread_local auto max_stream_duration = 10s;
                                  thread_local Mat rgb(Size(640, 480), CV_8UC3, Scalar(0));

                                  thread_local static auto run_stream = [&] {
                                      namedWindow("rgb");
                                      int key_interrupt = pollKey();
                                      while (key_interrupt < 0 &&
                                             std::chrono::duration_cast<std::chrono::seconds>(now - start) <= max_stream_duration) {
                                          now = high_resolution_clock::now();
                                          if (_rgb_stream.pop(rgb)) {
                                              cv::imshow("rgb", rgb);
                                          }

                                          key_interrupt = pollKey();
                                      }
                                      destroyAllWindows();
                                  };

                                  run_stream();
                              });
                          }),
            ftxui::Button(
                "4. Face Detection",
                [this] {
                    std::jthread([this] {
                        using namespace std::chrono;          // NOLINT
                        using namespace std::chrono_literals; // NOLINT
                        using namespace cv;                   // NOLINT

                        thread_local Mat rgb(Size(640, 480), CV_8UC3, Scalar(0));
                        thread_local Mat cascade_grayscale;
                        cv::CascadeClassifier face_cf{"/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml"};
                        static constexpr float cascade_image_scale{1.5F};
                        std::vector<cv::Rect> faces;

                        thread_local static auto run_stream = [&] {
                            namedWindow("Face Tracking");
                            int key_interrupt = pollKey();
                            //NOLINTBEGIN
                            const auto swidth = static_cast<int>(static_cast<float>(rgb.size().width) / cascade_image_scale);
                            const auto sheight = static_cast<int>(static_cast<float>(rgb.size().height) / cascade_image_scale);
                            //NOLINTEND
                            // Detect faces
                            while (key_interrupt < 0) {
                                if (_rgb_stream.pop(rgb)) {
                                    resize(rgb, cascade_grayscale, cv::Size(swidth, sheight));
                                    cvtColor(cascade_grayscale, cascade_grayscale, cv::COLOR_BGR2GRAY);
                                    face_cf.detectMultiScale(rgb, faces, 1.1, 3, 0, cv::Size(50, 50));
                                    if (!faces.empty()) {
                                        // Apply rectangles to BGR image
                                        for (const auto& face : faces) {
                                            cv::rectangle(
                                                cascade_grayscale,
                                                cv::Point(cvRound(static_cast<float>(face.x) * cascade_image_scale),
                                                          cvRound(static_cast<float>(face.y) * cascade_image_scale)), // Upper left point
                                                cv::Point(cvRound((static_cast<float>(face.x) + static_cast<float>(face.width - 1)) *
                                                                  cascade_image_scale),
                                                          cvRound((static_cast<float>(face.y) + static_cast<float>(face.height - 1)) *
                                                                  cascade_image_scale)), // Lower right point
                                                cv::Scalar(0, 0, 255)                    // Red line
                                            );
                                        }
                                        faces.clear();
                                    }
                                    cv::imshow("Face Detect", rgb);
                                    key_interrupt = pollKey();
                                }
                            }
                            destroyAllWindows();
                        };

                        run_stream();
                    });
                }),
            ftxui::Button(
                "5. Cascade Grayscale Body Detection",
                [this] {
                    std::jthread([this] {
                        using namespace std::chrono;          // NOLINT
                        using namespace std::chrono_literals; // NOLINT
                        using namespace cv;                   // NOLINT

                        thread_local Mat rgb(Size(640, 480), CV_8UC3, Scalar(0));
                        thread_local Mat cascade_grayscale;
                        thread_local cv::CascadeClassifier body_cf{"/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml"};
                        std::vector<cv::Rect> bodies;

                        thread_local static auto run_stream = [&] {
                            namedWindow("Cascade Grayscale Body Tracking");
                            int key_interrupt = pollKey();
                            //NOLINTBEGIN
                            const auto swidth = static_cast<int>(static_cast<float>(rgb.size().width) / cascade_image_scale);
                            const auto sheight = static_cast<int>(static_cast<float>(rgb.size().height) / cascade_image_scale);
                            //NOLINTEND
                            // Detect faces
                            while (key_interrupt < 0) {
                                if (_rgb_stream.pop(rgb)) {
                                    resize(rgb, cascade_grayscale, cv::Size(swidth, sheight));
                                    cvtColor(cascade_grayscale, cascade_grayscale, cv::COLOR_BGR2GRAY);
                                    body_cf.detectMultiScale(rgb, bodies, 1.1, 2, 1, cv::Size(40, 70)), Size(300, 200);
                                    if (!body_cf.empty()) {
                                        // Apply rectangles to BGR image
                                        for (const auto& body : bodies) {
                                            cv::rectangle(
                                                cascade_grayscale,
                                                cv::Point(cvRound(static_cast<float>(body.x) * cascade_image_scale),
                                                          cvRound(static_cast<float>(body.y) * cascade_image_scale)), // Upper left point
                                                cv::Point(cvRound((static_cast<float>(body.x) + static_cast<float>(body.width - 1)) *
                                                                  cascade_image_scale),
                                                          cvRound((static_cast<float>(body.y) + static_cast<float>(body.height - 1)) *
                                                                  cascade_image_scale)), // Lower right point
                                                cv::Scalar(0, 0, 255)                    // Red line
                                            );
                                        }
                                        bodies.clear();
                                    }
                                    cv::imshow("Cascade Grayscale Body Tracking", rgb);
                                    key_interrupt = pollKey();
                                }
                            }
                            destroyAllWindows();
                        };

                        run_stream();
                    });
                }),
            ftxui::Button(
                "6. Cascade Color Body Detection",
                [this] {
                    std::jthread([this] {
                        using namespace std::chrono;          // NOLINT
                        using namespace std::chrono_literals; // NOLINT
                        using namespace cv;                   // NOLINT

                        thread_local Mat rgb(Size(640, 480), CV_8UC3, Scalar(0));
                        thread_local cv::CascadeClassifier body_cf{"/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml"};
                        static constexpr float cascade_image_scale{1.5F};
                        std::vector<cv::Rect> bodies;

                        thread_local static auto run_stream = [&] {
                            namedWindow("Cascade Color Body Tracking");
                            int key_interrupt = pollKey();
                            // Detect faces
                            while (key_interrupt < 0) {
                                if (_rgb_stream.pop(rgb)) {
                                    body_cf.detectMultiScale(rgb, bodies, 1.1, 2, 1, cv::Size(40, 70)), Size(300, 200);
                                    if (!body_cf.empty()) {
                                        // Apply rectangles to BGR image
                                        for (const auto& body : bodies) {
                                            cv::rectangle(
                                                rgb,
                                                cv::Point(cvRound(static_cast<float>(body.x) * cascade_image_scale),
                                                          cvRound(static_cast<float>(body.y) * cascade_image_scale)), // Upper left point
                                                cv::Point(cvRound((static_cast<float>(body.x) + static_cast<float>(body.width - 1)) *
                                                                  cascade_image_scale),
                                                          cvRound((static_cast<float>(body.y) + static_cast<float>(body.height - 1)) *
                                                                  cascade_image_scale)), // Lower right point
                                                cv::Scalar(0, 0, 255)                    // Red line
                                            );
                                        }
                                        bodies.clear();
                                    }
                                    cv::imshow("Cascade Color Body Tracking", rgb);
                                    key_interrupt = pollKey();
                                }
                            }
                            destroyAllWindows();
                        };

                        run_stream();
                    });
                }),
            ftxui::Button("7. HOG Pedestrian Detection",
                          [this] {
                              std::jthread([this] {
                                  using namespace std::chrono;          // NOLINT
                                  using namespace std::chrono_literals; // NOLINT
                                  using namespace cv;                   // NOLINT

                                  thread_local Mat rgb(Size(640, 480), CV_8UC3, Scalar(0));
                                  static thread_local cv::HOGDescriptor hog_d(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9, 1, -1,
                                                                              HOGDescriptor::L2Hys, 0.2, true,
                                                                              cv::HOGDescriptor::DEFAULT_NLEVELS);
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
                                              hog_d.detectMultiScale(rgb, people, confidences, detection_threshold, Size(8, 8), Size(),
                                                                     1.05, 2, false);
                                              if (!people.empty()) {
                                                  // Apply rectangles to BGR image
                                                  auto confidence = confidences.begin();
                                                  for (auto person = people.begin(); person != people.end();
                                                       std::advance(person, 1), std::advance(confidence, 1)) {

                                                      cv::rectangle(rgb, person->tl(), person->br(), cv::Scalar(0, 255, 0), 2);
                                                      if (confidence != confidences.end()) {
                                                          cv::putText(rgb, absl::StrFormat("Confidence: %f", *confidence),
                                                                      Point(person->x, person->y), FONT_HERSHEY_SIMPLEX, 1.0,
                                                                      Scalar(255, 100, 0), 2);
                                                      }
                                                  }
                                                  people.clear();
                                              }
                                              cv::putText(rgb, absl::StrFormat("DThreshold: %f", detection_threshold), Point(5, 25),
                                                          FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 100, 0), 2);
                                              cv::imshow("Pedestrian Tracking", rgb);
                                              //UP: 1113938
                                              //DOWN: 1113940
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
                          }),
        });

      private: // funcs
        ftxui::Component _keyboard_renderer(ftxui::ScreenInteractive& screen) {
            using namespace ftxui; // NOLINT
            Component new_render;
            new_render = Renderer([&, this] {
                static auto state_doc = vflow({});
                Event ele_itr;
                double acc_x{};
                double acc_y{};
                double acc_z{};
                auto new_state = _kin.get_state(); // TODO fix not checking optional
                // new_state.getAccelerometers(&acc_x, &acc_y, &acc_z);
                auto accel_str = absl::StrFormat("Accelerometer\n\t X: %f, Y: %f, Z: %f\n", acc_x, acc_y, acc_z);
                auto tilt_str = absl::StrFormat("Angle (Degrees): %f\n", new_state.cmded_angle.value());

                state_doc = vflow({text("Tilt Motor State") | bgcolor(Color::Black) | color(Color::White) | bold,
                                   text(accel_str) | bgcolor(Color::White) | color(Color::Blue),
                                   text(tilt_str) | bgcolor(Color::White) | color(Color::Blue)});

                return vbox({
                    text("Press Q to go back"),
                    ftxui::separator(),
                    window(text("Kinect State"), state_doc),
                });
            });

            new_render |= CatchEvent([&screen, this](const Event& event) {
                static auto stringify = [](const Event& event) -> std::string {
                    if (event.is_character()) {
                        return event.character();
                    }
                    return "";
                };
                if (event.is_character()) {
                    auto str_input = stringify(event);
                    auto new_state = _kin.get_state();
                    if (!str_input.empty()) {
                        if (str_input.size() == 1) {
                            bool fast_rate{false};
                            switch (str_input.front()) {
                                case 'Q':
                                    [[fallthrough]];
                                case 'q': {
                                    screen.ResetPosition(true);
                                    screen.ExitLoopClosure()();
                                    break;
                                }
                                case 'P':
                                    [[fallthrough]];
                                case 'p': {

                                    break;
                                }
                                case 'S':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 's': {
                                    double dif =
                                        (fast_rate) ? kinect::units::tilt_properties::fast_step : kinect::units::tilt_properties::slow_step;
                                    auto new_ang = kinect::units::Degrees{new_state.cmded_angle.value() - dif};
                                    _kin.set_tilt(new_ang);
                                    break;
                                }
                                case 'W':
                                    fast_rate = true;
                                    [[fallthrough]];
                                case 'w': {
                                    double dif =
                                        (fast_rate) ? kinect::units::tilt_properties::fast_step : kinect::units::tilt_properties::slow_step;
                                    auto new_ang = kinect::units::Degrees{new_state.cmded_angle.value() + dif};
                                    _kin.set_tilt(new_ang);
                                } break;
                                case 'C':
                                case 'c': {
                                    auto new_color = (new_state.led_color == kinect::units::eLEDColors::GREEN)
                                                         ? kinect::units::eLEDColors::YELLOW
                                                         : kinect::units::eLEDColors::GREEN;
                                    _kin.set_color(new_color);
                                    break;
                                }
                                case 'R':
                                    [[fallthrough]];
                                case 'r': {
                                    _kin.set_tilt(kinect::units::Degrees{0.0});
                                }
                                default:
                                    break;
                            }
                        }
                    }

                    return true;
                }
                return false;
            });

            return new_render;
        }

        ftxui::Component _rgb_stream_renderer(ftxui::ScreenInteractive& screen) {
            using namespace ftxui; // NOLINT
            // A triangle following the mouse, using braille characters.
            Component renderer = Renderer([&] {
                auto c = Canvas(640, 480);
                cv::Mat rgb;
                if (_rgb_stream.pop(rgb)) {
                    std::mutex mtx;
                    auto* data = rgb.data;
                    tbb::parallel_for(tbb::blocked_range2d<int>(0, rgb.rows - 1, 0, rgb.cols - 1),
                                      [&](const tbb::blocked_range2d<int>& mat) {
                                          for (int i = mat.rows().begin(); i != mat.rows().end(); ++i) {
                                              for (int j = mat.cols().begin(); j != mat.cols().end(); ++j) {
                                                  uchar b = data[i * rgb.step + rgb.channels() * j + 0];
                                                  uchar g = data[i * rgb.step + rgb.channels() * j + 1];
                                                  uchar r = data[i * rgb.step + rgb.channels() * j + 2];
                                                  std::scoped_lock lck(mtx);
                                                  c.DrawPoint(j, i, true, Color::RGB(r, g, b));
                                              }
                                          }
                                      });

                } else {
                    c.DrawText(640 / 2, 480 / 2,

                               absl::StrFormat("No Stream (%i)", std::chrono::high_resolution_clock::now().time_since_epoch().count()),
                               [](Pixel& p) {
                                   p.foreground_color = Color::Red;
                                   p.underlined = true;
                                   p.bold = true;
                                   p.background_color = Color::Black;
                               });
                }
                screen.PostEvent(Event::Custom);
                return canvas(std::move(c));
            });

            return renderer;
        }

        void _rgb_stream_cb(const cv::Mat& rgb_mat) { _rgb_stream.push(rgb_mat); }
    };

} // namespace cerberus::terminal
#endif //__CERBERUS_TERMINAL_TUI_H_