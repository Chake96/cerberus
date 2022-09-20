#ifndef __CERBERUS_CAMERAS_MANAGER_H_
#define __CERBERUS_CAMERAS_MANAGER_H_

#include <cerberus/cameras/camera.h>
#include <cerberus/cameras/utils.h>
#include <cerberus/utilities/thread_pool.h>

#include <execution>
#include <future>
#include <memory>
#include <string_view>
#include <vector>

#include <libusb-1.0/libusb.h>
#include <source_location>
#include <spdlog/async.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <absl/container/flat_hash_set.h>
#include <absl/container/inlined_vector.h>
#include <absl/status/status.h>

namespace cerberus::cameras {

    class CameraManager {

      public:
        ~CameraManager();

        explicit CameraManager(utilities::threads::ThreadPool& tp);

        template <class CameraType = Camera>
        enable_if_camera_t<CameraType, std::shared_ptr<CameraType>> get(std::string_view model) {
            // auto cam = _cameras.find(model);
            auto cam = std::find(_cameras.begin(), _cameras.end(), [&](auto cam) { return cam->properties.model_name == model; });
            if (cam != _cameras.end()) {
                return std::dynamic_pointer_cast<CameraType>(*cam);
            }
            return nullptr;
        }

        template <class CameraType = Camera>
        enable_if_camera_t<CameraType, std::shared_ptr<CameraType>> get(ssize_t index) {
            // auto cam = _cameras.find(model);
            auto cam = std::next(_cameras.begin(), index);
            if (cam != _cameras.end()) {
                return std::dynamic_pointer_cast<CameraType>(*cam);
            }
            return nullptr;
        }

        absl::Status attach_cb_to_all(const std::function<void(cv::Mat)>& cb) {
            std::for_each(std::execution::par_unseq, _cameras.begin(), _cameras.end(), [&cb](auto camera) {
                camera->register_cv_mat_signal(cb);
            });
            return absl::OkStatus();
        }

      protected:
        // logging
        std::shared_ptr<spdlog::logger> _logger{nullptr};
        // your code for which the warning gets suppressed

        static constexpr std::source_location _source_loc = // NOLINT
            std::source_location::current();

      private:
        struct USBDevs {
            libusb_context* context{nullptr};
            libusb_device** devices{nullptr};
            ssize_t device_count{0};
            static constexpr size_t max_num_devices{70};
            absl::InlinedVector<libusb_device_descriptor, max_num_devices> descriptions;
            absl::InlinedVector<libusb_config_descriptor*, max_num_devices> configurations;
        } _usb_devs{};

        // absl::InlinedVector<std::shared_ptr<Camera>, 40> _cameras{};

        void _enumerate_usb_cams() {}

        utilities::threads::ThreadPool& _tpool;

        // std::set<std::shared_ptr<Camera>, Camera::Comparator> _cameras{};
        std::vector<std::shared_ptr<Camera>> _cameras{};

      private:
        static bool _check_known_usb_cams(libusb_device_descriptor const* description);

        void _init_usb_cameras();
        void _init_uvc_cameras();
        std::promise<void> _stop_init_threads_signal;
        std::shared_future<void> _stop_init_threads_fut{_stop_init_threads_signal.get_future()};
    };

} // namespace cerberus::cameras

#endif // __CERBERUS_CAMERAS_MANAGER_H_