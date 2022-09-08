#ifndef __CERBERUS_CAMERAS_USB_CAM_H_
#define __CERBERUS_CAMERAS_USB_CAM_H_

#include <cerberus/cameras/camera.h>
#include <cerberus/cameras/utils.h>
#include <cerberus/utilities/si_units.h>

#include <string_view>

#include <absl/status/status.h>

#include <fmt/format.h>

namespace cerberus::cameras::usb {

    class USBCamera : virtual public cerberus::cameras::Camera {
      public: //methods
        static constexpr camera_types type{camera_types::USB};

        USBCamera() = delete;
        USBCamera(uint32_t serial_num, std::string_view name, std::string_view usb_desc, cameras::codecs codec = codecs::H264_AVC) {}

        absl::Status zoom(units::si::Degrees deg) override {
            absl::Status ret;
            if (deg > properties.max_fov || deg < properties.min_fov) {

                ret = absl::OkStatus();
            } else {
                ret = absl::InvalidArgumentError(
                    fmt::format("Degrees outside Zoom boundaries of Camera {}", boost::uuids::to_string(properties.uuid))
                );
            }
            return ret;
        }

        absl::Status zoom(units::Magnification mag) override {
            auto m = mag; //NOLINT
            absl::Status ret;
            ret = absl::OkStatus();
            return ret;
        }

        absl::Status reset() override { return absl::OkStatus(); }

      private: //vars
        const std::string_view _simple_name;
        const std::string_view _usb_descriptor;
        const cameras::codecs _codec;
    };

} // namespace cerberus::cameras::usb

#endif //__CERBERUS_CAMERAS_USB_CAM_H_