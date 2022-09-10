#include "cerberus/cameras/camera.h"
#include <cerberus/cameras/manager.h>
#include <cerberus/cameras/usb_cam.h>

#include <libusb-1.0/libusb.h>

#include "boost/uuid/uuid_io.hpp"

namespace cerberus::cameras {

    namespace {
        enum class vendor_ids : uint16_t {
            LOGICTECH_VEN_ID = 0x046D,
        };

        enum class logitech_ids : uint16_t {
            C615 = 0x82C,
        };

    } // namespace

    CameraManager::~CameraManager() {
        size_t count{0};
        for (auto* config : _usb_devs.configurations) { // free using libusb
            try {
                libusb_free_config_descriptor(config);
            } catch (...) {
                _logger->debug("failed to Free Config #{}", count);
            }
            count += 1;
        }
        libusb_free_device_list(_usb_devs.devices, static_cast<int>(_usb_devs.device_count));
        libusb_exit(_usb_devs.context);
    }

    CameraManager::CameraManager(utilities::threads::ThreadPool& tp) : _tpool(tp) {

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

    bool CameraManager::_check_known_usb_cams(libusb_device_descriptor const* description) {
        bool is_usb_cam{false};
        const vendor_ids v_id{description->idVendor};
        switch (v_id) {
            case vendor_ids::LOGICTECH_VEN_ID: {
                const logitech_ids p_id{description->idProduct};
                switch (p_id) {
                    case logitech_ids::C615:
                        is_usb_cam = true;
                        break;
                    default:
                        break;
                }
                break;
            }
        }
        return is_usb_cam;
    }

    void CameraManager::_init_usb_cameras() {
        int result_code{0};
        result_code = libusb_init(&_usb_devs.context);
        if (result_code != LIBUSB_SUCCESS) {
            _logger->error(fmt::format("LibUSB Init Returned Code: {}", result_code));
        }
        _usb_devs.device_count = libusb_get_device_list(_usb_devs.context, &_usb_devs.devices);
        _logger->info("LIB USB libusb_get_device_list states there are {} USB devices total", _usb_devs.device_count);

        for (auto dev_count = _usb_devs.device_count - 1; dev_count >= 0; --dev_count) {
            auto* description = &_usb_devs.descriptions.emplace_back(0);
            auto* device = _usb_devs.devices[dev_count];
            // get device descriptors
            result_code = libusb_get_device_descriptor(device, description);
            if (result_code != LIBUSB_SUCCESS) {
                _logger->warn(fmt::format("Getting Device #{0}'s Descriptor Resulted in LIBUSB Error Code: {1}", dev_count, result_code));
            }

            if (description->idVendor == 0x1993 && (description->idProduct >= 0x8201 && description->idProduct <= 0x8208))
                continue;

            for (auto config_count = 0; config_count < description->bNumConfigurations; config_count++) {
                auto* config = _usb_devs.configurations.emplace_back();
                result_code = libusb_get_config_descriptor(device, config_count, &config);
                if (result_code != LIBUSB_SUCCESS) {
                    _logger->warn(
                        fmt::format("Getting Device #{0}'s Configurations Resulted in LIBUSB Error Code: {1}", dev_count, result_code)
                    );
                } else {
                    auto& descript = _usb_devs.descriptions.back();
                    _logger->debug(
                        fmt::format("Added USB Device #{0}: Vendor[{1}]:ID[{2}]", dev_count, descript.idVendor, descript.idProduct)
                    );
                    bool is_video_class = (descript.bDeviceClass == LIBUSB_CLASS_VIDEO) || _check_known_usb_cams(&descript);
                    bool class_defined_by_interface =
                        (descript.bDeviceClass == LIBUSB_CLASS_MISCELLANEOUS) && // defined here: https://www.usb.org/defined-class-codes
                        (descript.bDeviceSubClass == 0x02) && (descript.bDeviceProtocol == 0x01);

                    bool is_video_interface = false;
                    static constexpr uint8_t INTERFACE_VIDEO_CLASS = 14U;
                    static constexpr uint8_t INTERFACE_VIDEO_SUBCLASS = 2U;

                    if (class_defined_by_interface) {
                        // clang-format off
                            //TODO: clean up -- conditions should be made concise
                            auto interface_id = 0;
                            auto interface_loop_cond =[&]()->bool{
                                return !is_video_interface &&  (interface_id < config->bNumInterfaces || config->bNumInterfaces == 0);
                            };
                            for (; interface_loop_cond() ; ++interface_id) {
                                auto alt_set_id = 0;
                                auto setting_loop_cond = [&]()->bool{return !is_video_interface  && alt_set_id < config->interface[interface_id].num_altsetting;};
                                for (; setting_loop_cond(); ++alt_set_id) { 
                                    auto const* settings = &config->interface->altsetting[alt_set_id]; 
                                    if(settings->bInterfaceClass != 0 && settings->bInterfaceClass != 1){
                                        _logger->info(fmt::format("Class: {}, Subclass: {} ", settings->bInterfaceClass, settings->bInterfaceSubClass));
                                    }
                                    is_video_interface = (settings->bInterfaceClass == INTERFACE_VIDEO_CLASS || 
                                                            settings->bInterfaceSubClass == INTERFACE_VIDEO_SUBCLASS ||
                                                            ( 0x199e == descript.idVendor && ( 0x8101 == descript.idProduct ||
                                                                0x8102 == descript.idProduct ) &&
                                                                settings->bInterfaceClass == 255 &&
                                                                settings->bInterfaceSubClass == 2 )
                                                            ) ;
                                }
                            }
                        // clang-format on
                        // is_video_interface = config->interface->altsetting->bInterfaceClass
                    }

                    if (is_video_class || is_video_interface) {
                        int bus = libusb_get_bus_number(device);
                        int address = libusb_get_device_address(device);
                        int port = libusb_get_port_number(device);
                        _logger->info(fmt::format("Found USB Video Device on Bus {}:{} using Port[{}]", bus, address, port));
                        // TODO: most recent -- continue adding to camera registry
                        auto usb_cam = std::make_shared<usb::USBCamera>(_usb_devs.context, device, description);
                        usb_cam->connect_vid_cb([this](const std::vector<uint8_t>& vec) {
                            _logger->debug("Got USB Packet with Size {}", vec.size());
                        });
                        auto started_success = usb_cam->start_video();
                        if (!started_success.ok()) {
                            _logger->warn(
                                "Manager's USB Camera [UUID: {}] failed to start its video",
                                boost::uuids::to_string(usb_cam->properties.uuid)
                            );
                        }
                        // _cameras.insert(usb_cam);
                    }
                }
            }
        }
    }

} // namespace cerberus::cameras