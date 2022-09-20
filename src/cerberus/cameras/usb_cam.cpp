#include <cerberus/cameras/usb_cam.h>

#include <exception>

#include <libusb.h>
#include <opencv2/core/mat.hpp>
#include <spdlog/spdlog.h>

namespace cerberus::cameras::usb {

    namespace {

        auto init_description = [](libusb_device_descriptor * desc, libusb_device_handle* handle) -> auto{
            static constexpr size_t max_desc_len{256U};
            // static constexpr size_t serial_len{16U};
            unsigned char string[256];
            auto num_bytes = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber, string, max_desc_len);
            if (num_bytes < 0) {
                constexpr auto loc = std::source_location::current();
                std::cerr << fmt::format("{}:[Line: {}] LIBUSB returned: {}\n", loc.file_name(), loc.line(), num_bytes);
                num_bytes = 0;
            }
            std::string serial{std::begin(string), std::begin(string) + num_bytes};
            memset(string, 0, sizeof(string));
            num_bytes = libusb_get_string_descriptor_ascii(handle, desc->iManufacturer, string, max_desc_len);
            if (num_bytes < 0) {
                constexpr auto loc = std::source_location::current();
                std::cerr << fmt::format("{}:[Line: {}] LIBUSB returned: {}\n", loc.file_name(), loc.line(), num_bytes);
                num_bytes = 0;
            }
            std::string manufac{std::begin(string), std::begin(string) + num_bytes};
            memset(string, 0, sizeof(string));
            num_bytes = libusb_get_string_descriptor_ascii(handle, desc->iProduct, string, max_desc_len);
            if (num_bytes < 0) {
                constexpr auto loc = std::source_location::current();
                std::cerr << fmt::format("{}:[Line: {}] LIBUSB returned: {}\n", loc.file_name(), loc.line(), num_bytes);
                num_bytes = 0;
            }
            std::string product{std::begin(string), std::begin(string) + num_bytes};
            return USBDescription{
                .serial_number = serial,
                .manufacturer = manufac,
                .product = product,
                .vendor_id = desc->idVendor,
                .product_id = desc->idProduct,
            };
        };

        auto init_handle(libusb_device* dev, libusb_device_handle* handle) -> libusb_device_handle* {
            static size_t count{0};
            auto logger = spdlog::get(std::string{USBCamera::log_name});
            auto result_code = libusb_open(dev, &handle);
            if (logger) {
                if (result_code != LIBUSB_SUCCESS) {
                    logger->critical(fmt::format("Unable to open USB Device {} with error: {}", count, libusb_error_name(result_code)));
                } else {
                    logger->info(fmt::format("Succesfully opened USB Device {}", count));
                }
            }
            count += 1;
            return handle;
        };

        auto init_iso_pckt_size(const libusb_device* dev, int backup_size = 0) -> int {
            auto max_iso_sz = libusb_get_max_iso_packet_size(const_cast<libusb_device*>(dev), 0x81);
            if (max_iso_sz < 0) {
                auto logger = spdlog::get(std::string{USBCamera::log_name});
                logger->warn(fmt::format(
                    "[LIBUSB Error]: Unable to get Max ISO Packet Size, returned: {} which maps to error {}\n",
                    max_iso_sz,
                    libusb_error_name(max_iso_sz)
                ));
                max_iso_sz = backup_size; // TODO: determine which of these is correct
            }
            return max_iso_sz;
        }

        boost::signals2::signal<void(std::vector<uint8_t>, libusb_device_handle*)> global_buffer{}; // UUID as key

        void LIBUSB_CALL global_buffer_cb(libusb_transfer* transfer) {
            std::cerr << "Debugin\n";
            switch (transfer->status) {
                case LIBUSB_TRANSFER_COMPLETED: {
                    thread_local std::vector<uint8_t> temp_buf{};
                    temp_buf.clear();
                    for (auto byte = 0; byte < transfer->actual_length; ++byte) {
                        temp_buf.push_back(transfer->buffer[byte]);
                    }
                    global_buffer(temp_buf, transfer->dev_handle);
                    auto submit_status = libusb_submit_transfer(transfer);
                    if (submit_status != LIBUSB_SUCCESS) {}
                    break;
                }
                // case LIBUSB_TRANSFER_ERROR:
                //     [[fallthrough]];
                // case LIBUSB_TRANSFER_TIMED_OUT:
                //     [[fallthrough]];
                // case LIBUSB_TRANSFER_CANCELLED:
                //     [[fallthrough]];
                // case LIBUSB_TRANSFER_STALL:
                //     [[fallthrough]];
                // case LIBUSB_TRANSFER_NO_DEVICE:
                //     [[fallthrough]];
                // case LIBUSB_TRANSFER_OVERFLOW:
                default:
                    libusb_free_transfer(transfer);
                    exit(3);
                    break;
            }
        }
    } // namespace

    USBCamera::~USBCamera() {
        libusb_release_interface(_dev_handle, 3);
        libusb_close(_dev_handle);
        _dev_handle = nullptr;
    }

// LINTBUG: remove pragmas, initalize _dev_handle to something
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"

#else
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wuninitialized"
#endif
    // #pragma clang diagnostic ignore "-Wuninitizaled"
    USBCamera::USBCamera(libusb_context* ctx, libusb_device* usb_dev, libusb_device_descriptor* usb_desc)
        : Camera::Camera({}, {}),
          _usb_dev(usb_dev),
          _dev_handle(init_handle(const_cast<libusb_device*>(_usb_dev), _dev_handle)),
          description(init_description(usb_desc, _dev_handle)),
          _codec(),
          _ctx(ctx), // NOLINT(-Wuninitialized)
          _usb_descript(usb_desc),
          _usb_img_transfer(libusb_alloc_transfer(16)),
          _max_iso_pckt_sz(init_iso_pckt_size(_usb_dev, 16)) { // std::max(properties.resolution.height, properties.resolution.width))) {

        // auto err_c = libusb_set_auto_detach_kernel_driver(_dev_handle, static_cast<int>(true));
        // if (err_c != LIBUSB_SUCCESS) {
        //     _logger->warn("This USB Device does not support Auto-Detach ->", libusb_error_name(err_c));
        // }

        // err_c = libusb_claim_interface(_dev_handle, 3);

        // if (err_c != LIBUSB_SUCCESS) {
        //     _logger->error("Failed to Claim Interface with Code: {}", libusb_error_name(err_c));
        // }

        // if (err_c != LIBUSB_SUCCESS) {
        //     _logger->error("Failed to Set Interface Alternatives with Code: {}", libusb_error_name(err_c));
        // }
        // libusb_set_iso_packet_lengths(_usb_img_transfer, _max_iso_pckt_sz);
        global_buffer.connect([this](auto vec, auto* handle) {
            if (handle == _dev_handle) { // only call the signal when we identify its this instance's device
                _call_vid_cb(vec);
            }
        });
    }
// LINTBUG: remove compiler specific diagnostics
#ifdef __GNUC__
#pragma GCC diagnostic pop
#else
#pragma clang diagnostic pop
#endif

    absl::Status USBCamera::start_video() {
        static constexpr uint8_t xfrs{1}, pckts{16}; // NOLINT
        static uint8_t byte_buf[192 * 16];

        libusb_fill_iso_transfer(
            _usb_img_transfer,
            _dev_handle,
            0x81, // EP_DATA
            byte_buf,
            sizeof(byte_buf),
            16 * 192,
            global_buffer_cb, // NOLINT
            nullptr,
            5000
        );

        libusb_set_iso_packet_lengths(_usb_img_transfer, sizeof(byte_buf)); // REVISIT: sets packet length?

        if (libusb_kernel_driver_active(_dev_handle, 3) == 1) { // find out if kernel driver is attached
            if (libusb_detach_kernel_driver(_dev_handle, 3) == 0)
                _logger->error("Detached the Kernel Driver");
        }
        auto err_c = libusb_claim_interface(_dev_handle, 3);
        if (err_c != LIBUSB_SUCCESS) {
            _logger->error("Failed to ReClaim Interface with Code: {}", libusb_error_name(err_c));
        }
        err_c = libusb_set_interface_alt_setting(_dev_handle, 3, 1);
        if (err_c != LIBUSB_SUCCESS) {
            _logger->error("Failed to Set Alts with Code: {}", libusb_error_name(err_c));
        }

        absl::Status video_started = absl::OkStatus();
        auto transfer_succesful = libusb_submit_transfer(_usb_img_transfer);
        if (transfer_succesful != LIBUSB_SUCCESS) {
            video_started =
                absl::AbortedError(fmt::format("[INTERNAL ERROR] Failed to submit iso transfer, returned: {}", transfer_succesful));
            _logger->error(fmt::format(
                "USB Camera[{}] failed to submit it's iso tranfer, returned: {}",
                description.product,
                // boost::uuids::to_string(properties.uuid),
                libusb_error_name(transfer_succesful)
            ));
        }
        return video_started;
    }

    boost::signals2::connection USBCamera::connect_vid_cb(const std::function<void(std::vector<uint8_t>)>& callback) {
        return _call_vid_cb.connect(callback); // NOLINT
    }

} // namespace cerberus::cameras::usb