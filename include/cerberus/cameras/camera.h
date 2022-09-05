#ifndef __CERBERUS_CAMERAS_USB_CAM_H_
#define __CERBERUS_CAMERAS_USB_CAM_H_

#include <string_view>

#include <SI/illuminance.h>
#include <units/generic/angle.h>

#include <absl/status/status.h>

namespace cerberus::cameras {

    using Magnification = const double;
    using FOV = const double;

    class Camera {
      public: //variables
        const struct Properties { units::degree max_fov; } properties;

      public: //methods
        Camera& operator=(const Camera& other) = delete;
        Camera(const Camera& other) = delete;
        Camera& operator=(Camera&& other) = default;
        Camera(Camera&& other) = default;
        ~Camera() = default;

        virtual absl::Status zoom(const FOV& fov) = 0;
        virtual absl::Status zoom(Magnification mag) = 0;
        virtual absl::Status reset() = 0;
    };

} // namespace cerberus::cameras

#endif //__CERBERUS_CAMERAS_USB_CAM_H_