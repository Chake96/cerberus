#include "cerberus/cameras/uvc_cam.h"

#include "cerberus/cameras/camera.h"
#include "cerberus/cameras/utils.h"
#include "cerberus/utilities/c_callback.h"

#include <cstdio>
#include <exception>
#include <functional>
#include <iostream>
#include <istream>
#include <sstream>
#include <stdexcept>

#include <libuvc/libuvc.h>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include <fmt/core.h>

namespace cerberus::cameras::uvc {

    namespace {

        Properties init_properties(const UVCContext& dev) {
            uvc_device_descriptor_t* description;
            if (uvc_get_device_descriptor(dev.dev, &description) != UVC_SUCCESS) {
                fmt::print(stderr, "Failed to get Device Description");
                description = new uvc_device_descriptor_t;
            }

            Properties props{
                .fov = {},
                .model_name = description->product,
                .serial_number = description->serialNumber,
                .manufacturer = ((description->manufacturer != nullptr) ? description->manufacturer : kBadUsbStrDescription),
            };
            uvc_free_device_descriptor(description);
            return props;
        }

        // void bgr_frame_handler(uvc_frame* frame, void* user_data) {
        //     auto* bytes = static_cast<uint8_t*>(frame->data);
        //     auto* cam = static_cast<UVCCamera*>(user_data);

        //     std::copy(bytes, bytes + frame->data_bytes, _data_buf.begin());
        //     cv::Mat rgb(cv::Size(static_cast<int>(frame->width), static_cast<int>(frame->height)), CV_8UC3, cv::Scalar(0));
        //     rgb.data = bytes;
        //     cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
        //     cam->_cv_mat_signal(std::move(rgb));
        // }
    } // namespace

    UVCCamera::~UVCCamera() {
        uvc_stop_streaming(_uvc.handle);
        uvc_close(_uvc.handle);
        uvc_unref_device(_uvc.dev);
        uvc_exit(_uvc.ctx);
    }

    UVCCamera::UVCCamera(const UVCContext& _dev_context)
        : Camera::Camera(init_properties(_dev_context)), _uvc(_dev_context), _codec(cameras::codecs::MP4), _data_buf(buf_sz) {
        _logger->flush_on(spdlog::level::trace);

        cerberus::utilities::CCallback<void(uvc_frame*, void*)>::func = [this](auto&& PH1, auto&& PH2) {
            _handler(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
        };

        _c_frame_cb = static_cast<uvc_frame_callback_t*>(utilities::CCallback<void(uvc_frame*, void*)>::callback);

        if (uvc_start_streaming(_uvc.handle, &_uvc.ctrl, _c_frame_cb, nullptr, 0) != UVC_SUCCESS) {
            _logger->error("{} Failed to Video Stream", properties.model_name);
        }
        _logger->debug("UVC Device Opened\n");
    }

} // namespace cerberus::cameras::uvc