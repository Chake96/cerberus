#ifndef __CERBERUS_CAMERAS_MANAGER_H_
#define __CERBERUS_CAMERAS_MANAGER_H_

#include <cerberus/cameras/camera.h>
#include <cerberus/cameras/utils.h>
#include <cerberus/utilities/thread_pool.h>

#include <memory>
#include <string_view>

#include <libusb-1.0/libusb.h>
#include <source_location>
#include <spdlog/async.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <absl/container/flat_hash_set.h>
#include <absl/container/inlined_vector.h>
namespace cerberus::cameras {

    class CameraManager {

      public:
        ~CameraManager() {
            size_t count{0};
            for (auto* config : _usb_devs.configurations) { //free using libusb
                try {
                    libusb_free_config_descriptor(config);
                } catch (...) {
                    // _logger->error("failed to Free Config #{}", count);
                }
                count += 1;
            }
            libusb_free_device_list(_usb_devs.devices, static_cast<int>(_usb_devs.device_count));
            libusb_exit(_usb_devs.context);
        }

        explicit CameraManager(utilities::threads::ThreadPool& tp) : _tpool(tp) {

            _logger = spdlog::basic_logger_mt<spdlog::async_factory>("Camera Manager", "logs/camera_manager.txt");
            _logger->flush_on(spdlog::level::info);

            if (!_logger) {
                std::cerr << fmt::format(
                    "[{0}(Line {1}:{2}) {3}]: Failed to initalize Logger for CameraManager\n",
                    _source_loc.file_name(),
                    _source_loc.line(),
                    _source_loc.column(),
                    _source_loc.function_name()
                );
            }

            _init_usb_cameras();
            // _tpool.enqueue_asio(const Func& f)
        }

        template <class CameraType>
        enable_if_camera_t<CameraType, std::shared_ptr<CameraType>> get(std::string_view model) {
            auto cam = _cameras.find(model);
            if (cam != _cameras.end()) {
                return std::dynamic_pointer_cast<CameraType>(*cam);
            }
            return nullptr;
        }

      protected:
        //logging
        std::shared_ptr<spdlog::logger> _logger{nullptr};
        static constexpr std::source_location _source_loc = std::source_location::current();

      private:
        struct USBDevs {
            libusb_context* context{nullptr};
            libusb_device** devices{nullptr};
            ssize_t device_count{0};
            static constexpr size_t max_num_devices{70};
            absl::InlinedVector<libusb_device_descriptor, max_num_devices> descriptions;
            absl::InlinedVector<libusb_config_descriptor*, max_num_devices> configurations;
        } _usb_devs;

        // absl::InlinedVector<std::shared_ptr<Camera>, 40> _cameras{};

        void _enumerate_usb_cams() {}

        utilities::threads::ThreadPool& _tpool;

        std::set<std::shared_ptr<cerberus::cameras::Camera>, cerberus::cameras::Camera::Comparator> _cameras;

      private:
        static bool _check_known_usb_cams(libusb_device_descriptor const* description);

        void _init_usb_cameras();
    };

} // namespace cerberus::cameras

#endif // __CERBERUS_CAMERAS_MANAGER_H_