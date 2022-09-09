#ifndef __CERBERUS_CAMERAS_USB_CAM_H_
#define __CERBERUS_CAMERAS_USB_CAM_H_

#include <cerberus/cameras/camera.h>
#include <cerberus/cameras/utils.h>
#include <cerberus/utilities/si_units.h>

#include <functional>
#include <string_view>

#include <libusb-1.0/libusb.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <absl/status/status.h>

#include "boost/lockfree/spsc_queue.hpp"
#include "boost/uuid/uuid_io.hpp"

#include <fmt/format.h>

namespace cerberus::cameras::usb {

    struct USBDescription {
        const std::string serial_number, manufacturer, product;
        const uint16_t vendor_id, product_id;
    };

    class USBCamera : virtual public cerberus::cameras::Camera {
      public: // methods
        static constexpr camera_types type{camera_types::USB};

        ~USBCamera();
        USBCamera() = delete;
        USBCamera(libusb_context* ctx, libusb_device* usb_dev, libusb_device_descriptor* usb_desc);

        absl::Status stop_video() override { return absl::OkStatus(); }

        absl::Status start_video() override {
            static uint8_t byte_buf[1023];
            libusb_fill_iso_transfer(
                _usb_img_transfer,
                _dev_handle,
                (2 | LIBUSB_ENDPOINT_IN), // EP_DATA
                byte_buf,
                sizeof(byte_buf),
                25,
                std::bind_front(Fn && fn, Args && args...),
                nullptr,
                0
            );

            absl::Status video_started = absl::OkStatus();
            if (libusb_submit_transfer(_usb_img_transfer) != LIBUSB_SUCCESS) {
                video_started = absl::AbortedError("[INTERNAL ERROR] Failed to submit iso transfer");
                _logger->error(fmt::format(
                    "USB Camera[UUID: {}, Serial #: {}] failed to submit it's iso tranfer",
                    boost::uuids::to_string(properties.uuid),
                    properties.serial_number
                ));
            }
            return video_started;
        }

        absl::Status zoom(units::si::Degrees deg) override {
            absl::Status ret;
            if (deg > properties.fov.max || deg < properties.fov.min) {

                ret = absl::OkStatus();
            } else {
                ret = absl::InvalidArgumentError(
                    fmt::format("Degrees outside Zoom boundaries of Camera {}", boost::uuids::to_string(properties.uuid))
                );
            }
            return ret;
        }

        absl::Status zoom(units::Magnification mag) override {
            auto m = mag; // NOLINT
            absl::Status ret;
            ret = absl::OkStatus();
            return ret;
        }

        absl::Status reset() override { return absl::OkStatus(); }

      private: // vars
        const cameras::codecs _codec;
        const libusb_device* _usb_dev;
        const libusb_context* _ctx;
        const libusb_device_descriptor* _usb_descript;
        libusb_device_handle* _dev_handle;
        libusb_transfer* _usb_img_transfer;
        const int _max_iso_pckt_sz;
        boost::lockfree::spsc_queue<std::vector<uint8_t>> _recvd_pckts;
        const USBDescription _description;

      private: // methods
        void _transfer_buf_cb(libusb_transfer* transfer);

        void _pckt_handler_worker() {
            std::vector<uint8_t> pckt;
            if (_recvd_pckts.pop(pckt)) {
                // convert into open cv matrix and fire off the consumers
                cv::Mat vid_frame{properties.resolution.width, properties.resolution.height, CV_8UC3, cv::Scalar(0)};
                // auto* rgb_data = static_cast<uint8_t*>(pckt.data());
                vid_frame.data = pckt.data();
                cv::cvtColor(
                    vid_frame,
                    vid_frame,
                    cv::COLOR_RGB2BGR
                ); // TODO: review if this conversion is necessary based on Kinect's ordering of RGB stream
                _cv_mat_signal(vid_frame);
            } else {
                _logger->debug("Failed to Parse Packet from SPSC Queue");
            }
        }
    };

} // namespace cerberus::cameras::usb

#endif //__CERBERUS_CAMERAS_USB_CAM_H_