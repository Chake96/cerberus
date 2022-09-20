#ifndef __CERBERUS_CAMERAS_UVC_CAM_H_
#define __CERBERUS_CAMERAS_UVC_CAM_H_

#include <cerberus/cameras/camera.h>
#include <cerberus/cameras/usb_cam.h>
#include <cerberus/cameras/utils.h>
#include <cerberus/utilities/si_units.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include <libuvc/libuvc.h>
#undef UVC_DEBUG
#undef UVC_ENTER
#undef UVC_EXIT_VOID
#undef UVC_EXIT
#define UVC_DEBUG(format, ...)
#define UVC_ENTER()
#define UVC_EXIT_VOID()
#define UVC_EXIT(code)
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <absl/status/status.h>

#include <boost/lockfree/spsc_queue.hpp>

namespace cerberus::cameras::uvc {
    struct UVCContext {
        uvc_context_t* ctx;
        uvc_device_t* dev;
        uvc_device_handle_t* handle;
        uvc_stream_ctrl_t ctrl;
        uvc_error current_err;
    };

    class UVCCamera : virtual public cerberus::cameras::Camera {

      public: // methods
        static constexpr camera_types type{camera_types::UVC};
        enum class frame_types { RGB, BGR };

        UVCCamera() = delete;
        explicit UVCCamera(const UVCContext& _dev_context);

        ~UVCCamera() override;

        absl::Status stop_video() override { return absl::OkStatus(); }

        absl::Status start_video() override { return absl::OkStatus(); }

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

      protected:
        void _handler(uvc_frame* frame, void* user_data) {
            auto* bgr = uvc_allocate_frame(frame->width * frame->height * 3);
            if (bgr == nullptr)
                return;

            if (uvc_any2bgr(frame, bgr) != UVC_SUCCESS) {
                uvc_free_frame(bgr);
                return;
            }

            auto* bytes = static_cast<uint8_t*>(bgr->data);

            std::copy(bytes, bytes + bgr->data_bytes, _data_buf.begin());
            cv::Mat rgb(cv::Size(static_cast<int>(frame->width), static_cast<int>(frame->height)), CV_8UC3, cv::Scalar(0));
            rgb.data = bytes;
            // cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
            _cv_mat_signal(std::move(rgb));
        }
        uvc_frame_callback_t* _c_frame_cb{nullptr};

      private: // vars
        UVCContext _uvc;
        const cameras::codecs _codec;
        static constexpr size_t buf_sz = 1920 * 1080;
        std::vector<uint8_t> _data_buf;

      private: // methods
        friend void bgr_frame_handler(uvc_frame* frame, void* user_data);
    };

} // namespace cerberus::cameras::uvc

#endif // __CERBERUS_CAMERAS_UVC_CAM_H_