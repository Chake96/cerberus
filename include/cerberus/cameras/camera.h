#ifndef __CERBERUS_CAMERAS_CAMERA_H_
#define __CERBERUS_CAMERAS_CAMERA_H_

#include <cerberus/utilities/si_units.h>
#include <cerberus/utilities/units.h>

#include <string_view>

#include <numbers>

#include <absl/status/status.h>

#include "boost/uuid/random_generator.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace cerberus::cameras {

    struct Properties {
        Properties(cerberus::units::si::Degrees max, cerberus::units::si::Degrees min)
            : max_fov(max), min_fov(min), uuid(boost::uuids::random_generator_mt19937()()) {}
        const cerberus::units::si::Degrees max_fov{}, min_fov{};
        const boost::uuids::uuid uuid;
        const std::string_view model_name, serial_number;

        //Required Operators are Defaulted
        Properties& operator=(Properties&&) = default;
        Properties& operator=(const Properties&) = default;
        Properties(const Properties&) = default;
        Properties(Properties&&) = default;
    };

    class Camera {
      public: //variables
        const Properties properties;

        struct Comparator {
            using is_transparent = void;
            bool operator()(const std::shared_ptr<cerberus::cameras::Camera>& lhs, const std::shared_ptr<cerberus::cameras::Camera>& rhs) {
                return lhs->properties.uuid < rhs->properties.uuid;
            }
            bool operator()(const std::shared_ptr<cerberus::cameras::Camera>& lhs, std::string_view rhs) {
                return lhs->properties.model_name < rhs;
            }
            bool operator()(std::string_view lhs, const std::shared_ptr<cerberus::cameras::Camera>& rhs) {
                return lhs < rhs->properties.model_name;
            }
        };

      public: //methods
        //non-copyable
        Camera& operator=(const Camera& other) = delete;
        Camera(const Camera& other) = delete;

        //moveable
        Camera& operator=(Camera&& other) noexcept = default;
        Camera(Camera&& other) noexcept = default;

        Camera(cerberus::units::si::Degrees maxfov, cerberus::units::si::Degrees minfov) : properties(maxfov, minfov) {}
        ~Camera() = default;

        virtual absl::Status zoom(units::si::Degrees) = 0;
        virtual absl::Status zoom(units::Magnification) = 0;
        virtual absl::Status reset() = 0;

        auto operator==(const Camera& other) const { return properties.uuid == other.properties.uuid; }
        auto operator==(const std::string_view& other) const { return properties.model_name == other; }
        auto operator<(const Camera& other) const { return properties.uuid < other.properties.uuid; }
        auto operator<(const std::string_view& other) const { return properties.model_name == other; }
    };

} // namespace cerberus::cameras

#endif //__CERBERUS_CAMERAS_CAMERA_H_