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
            libusb_free_device_list(_usb_devs.devices, static_cast<int>(_usb_devs.device_count));
            libusb_exit(_usb_devs.context);
        }

        explicit CameraManager(utilities::threads::ThreadPool& tp) : _tpool(tp) {

            _logger = spdlog::basic_logger_mt<spdlog::async_factory>("Camera Manager", "logs/camera_manager.txt");

            if (!_logger) {
                std::cerr << fmt::format(
                    "[{0}(Line {1}:{2}) {3}]: Failed to initalize Logger for CameraManager\n",
                    _source_loc.file_name(),
                    _source_loc.line(),
                    _source_loc.column(),
                    _source_loc.function_name()
                );
            }
            int result_code{0};
            result_code = libusb_init(&_usb_devs.context);
            if (result_code != LIBUSB_SUCCESS) {
                _logger->error(fmt::format("LibUSB Init Returned Code: {}", result_code));
            }
            _usb_devs.device_count = libusb_get_device_list(_usb_devs.context, &_usb_devs.devices);
            _logger->info("LIB USB libusb_get_device_list states there are {} USB devices", _usb_devs.device_count);

            for (auto dev_count = _usb_devs.device_count - 1; dev_count >= 0; --dev_count) {
                auto* description = &_usb_devs.descriptions.emplace_back(0);
                auto* device = _usb_devs.devices[dev_count];
                result_code = libusb_get_device_descriptor(device, description);
                if (result_code != LIBUSB_SUCCESS) {
                    _logger->warn(fmt::format("Getting Device #{0}'s Descriptor Resulted in LIBUSB Error Code: {1}", dev_count, result_code)
                    );
                } else {
                    auto& descript = _usb_devs.descriptions.back();
                    _logger->debug(
                        fmt::format("Added USB Device #{0}: Vendor[{1}]:ID[{2}]", dev_count, descript.idVendor, descript.idProduct)
                    );
                }
            }
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
        } _usb_devs;
        void _enumerate_usb_cams() {}

        utilities::threads::ThreadPool& _tpool;

        std::set<std::shared_ptr<cerberus::cameras::Camera>, cerberus::cameras::Camera::Comparator> _cameras;
    };

} // namespace cerberus::cameras

#endif // __CERBERUS_CAMERAS_MANAGER_H_