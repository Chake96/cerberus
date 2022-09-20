#ifndef __CERBERUS_CAMERAS_CAMERA_H_
#define __CERBERUS_CAMERAS_CAMERA_H_

#include <cerberus/utilities/si_units.h>
#include <cerberus/utilities/units.h>

#include <memory>
#include <numbers>
#include <optional>
#include <string_view>
#include <utility>

#include <SI/length.h>
#include <libusb-1.0/libusb.h>
#include <opencv2/core/mat.hpp>
#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <absl/status/status.h>

#include <boost/signals2.hpp>
#include <boost/signals2/connection.hpp>
#include <boost/uuid/basic_name_generator.hpp>
#include <boost/uuid/name_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace cerberus::cameras {

    struct FOVProperties {
        const cerberus::units::si::Degrees max, min, step_size;
    };

    struct Resolution {
        const uint16_t width{}, height{};
    };

    struct Properties {
        const FOVProperties fov;
        const Resolution resolution;
        const std::string model_name, serial_number, manufacturer;
        const boost::uuids::uuid uuid{boost::uuids::name_generator_latest(boost::uuids::ns::oid())(model_name.data(), model_name.length())};
        const std::optional<units::si::Millimeters> focal_length;
    };

    class Camera {
      public: // variables
        const Properties properties;
        static constexpr std::string_view log_name{"Camera Logger"}, log_file{"logs/camera_logs.txt"};

        struct Comparator {
            using is_transparent = void;
            bool operator()(const std::shared_ptr<cerberus::cameras::Camera>& lhs, const std::shared_ptr<cerberus::cameras::Camera>& rhs) {
                return lhs->properties.uuid < rhs->properties.uuid;
            }
            bool operator()(const std::shared_ptr<cerberus::cameras::Camera>& lhs, std::string_view rhs) {
                return lhs->properties.model_name < rhs;
            }
            bool operator()(const std::shared_ptr<cerberus::cameras::Camera>& lhs, std::shared_ptr<cerberus::cameras::Camera>& rhs) {
                return lhs < rhs;
            }
            bool operator()(std::string_view lhs, const std::shared_ptr<cerberus::cameras::Camera>& rhs) {
                return lhs < rhs->properties.model_name;
            }
            template <class T>
            bool operator()(const T& lhs, const T& rhs) {
                return lhs < rhs;
            }
        };

      public: // methods
        // non-copyable
        Camera& operator=(const Camera& other) = delete;
        Camera(const Camera& other) = delete;

        // moveable
        Camera(Camera&& other) noexcept = default;

        virtual ~Camera() = default;
        explicit Camera(Properties properties) : properties(std::move(properties)) {
            _logger->flush_on(spdlog::level::level_enum::info);
            _logger->trace("Created new Camera -> {}", properties.model_name);
        }
        Camera(cerberus::units::si::Degrees /*maxfov*/, cerberus::units::si::Degrees /*minfov*/)
            : properties(Properties{.fov = FOVProperties{}, .resolution = {0, 0}, .model_name = ""}) {
            _logger->flush_on(spdlog::level::level_enum::info);
            _logger->trace("Created new Camera -> {}", properties.model_name);
        }

        virtual absl::Status stop_video() = 0;
        virtual absl::Status start_video() = 0;
        virtual absl::Status zoom(units::si::Degrees) = 0;
        virtual absl::Status zoom(units::Magnification) = 0;
        virtual absl::Status reset() = 0;

        // clang-format off
        boost::signals2::connection register_cv_mat_signal(const std::function<void(cv::Mat)>& cb) { // produces open CV Matrixs from the cameras video stream
            return _cv_mat_signal.connect(cb);
        }
        // clang-format on

        auto operator==(const Camera& other) const { return properties.uuid == other.properties.uuid; }
        auto operator==(const std::string_view& other) const { return properties.model_name == other; }
        auto operator<(const Camera& other) const { return properties.uuid < other.properties.uuid; }
        auto operator<(const std::string_view& other) const { return properties.model_name == other; }
        template <typename T>
        bool operator()(const T& lhs, const T& rhs) const {
            return lhs < rhs;
        };
        bool operator()(const std::shared_ptr<Camera>& lhs, const std::shared_ptr<Camera>& rhs) const { return lhs < rhs; };

      protected:
        // NOLINTBEGIN //LINTBUG: remove when bug in clang-tidy is fixed
        inline static std::shared_ptr<spdlog::logger> _logger = // NOLINT //LINTBUG: remove once fixed in clang
            spdlog::create_async<spdlog::sinks::basic_file_sink_mt>("Camera Logger", "logs/camera_logs.txt");
        boost::signals2::signal<void(cv::Mat)> _cv_mat_signal;
        // NOLINTEND

      private:
        static void _transfer_buf_cb(libusb_transfer* transfer);
    };

} // namespace cerberus::cameras

#endif //__CERBERUS_CAMERAS_CAMERA_H_