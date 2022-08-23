
#include <boost/signals2.hpp>

#include <kinect.h>

namespace cerberus::kinect{


    class KinectManager{
        public:
            using CallBackT = std::function<void (size_t)>;

            KinectManager() = default;

            size_t count() const{
                return _kinects.size();
            }
            

        private: //vars
            std::set<Kinect> _kinects{};

            std::vector<std::function<void (int8_t)>>

    };

}