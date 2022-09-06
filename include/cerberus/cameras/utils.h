#ifndef __CERBERUS_CAMERAS_UTILS_H_
#define __CERBERUS_CAMERAS_UTILS_H_

#include <cerberus/cameras/camera.h>

#include <string_view>
#include <type_traits>

namespace cerberus::cameras {
    enum class codecs { H264_AVC, H265_HEVC, MP4, AV1, VP9 };

    template <class CameraType, typename RetType = void>
    using enable_if_camera_t = std::enable_if_t<std::is_base_of_v<Camera, CameraType>, RetType>;

} // namespace cerberus::cameras

#endif //__CERBERUS_CAMERAS_UTILS_H_