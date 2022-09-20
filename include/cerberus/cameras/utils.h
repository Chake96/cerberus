#ifndef __CERBERUS_CAMERAS_UTILS_H_
#define __CERBERUS_CAMERAS_UTILS_H_

#include <cerberus/cameras/camera.h>

#include <string_view>
#include <type_traits>

namespace cerberus::cameras {
    enum class codecs { H264_AVC, H265_HEVC, MP4, AV1, VP9 };

    enum class camera_types { USB, UVC };

    template <class CameraType, typename RetType = void>
    using enable_if_camera_t = std::enable_if_t<std::is_base_of_v<Camera, CameraType>, RetType>;

    // NOLINTBEGIN(readability-identifier-naming)
    template <class CameraType, typename = void>
    struct camera_is_strong_typed : std::false_type {}; // NOLINT

    template <class CameraType>
    struct camera_is_strong_typed<CameraType, std::void_t<decltype(CameraType::type)>> : std::true_type {};

    template <class CameraType>
    using camera_is_strong_typed_v = typename camera_is_strong_typed<CameraType>::value;

    static const std::string kBadUsbStrDescription = std::string("BAD STRING DESCRIPTION FROM USB CAMERA");

    // NOLINTEND(readability-identifier-naming)
} // namespace cerberus::cameras

#endif //__CERBERUS_CAMERAS_UTILS_H_