#ifndef __CERBERUS_KINECT_H_
#define __CERBERUS_KINECT_H_


#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect.hpp>

#include <absl/strings/str_format.h>

#include <mutex>
#include <opencv2/core/mat.hpp>

#include <boost/signals2.hpp>

#include <SI/mass.h>
#include <SI/area.h>
#include <SI/angle.h>

#include <functional>
#include <cstdint>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <optional>
#include <thread>
#include <future>
#include <chrono>
#include <shared_mutex>
#include <mutex>

#include <type_traits>





namespace cerberus::kinect{


    const SI::kilo_gram_t<long double> kg{0};
    namespace units{
        enum class eLEDColors{OFF = 0, GREEN, RED, YELLOW, BLINK_GREEN, BLINK_R_Y};
        namespace tilt_properties{
            static constexpr double fast_step = 5, slow_step = 2;
            static constexpr double epsilon{0.25};
            enum class status{STOPPED, AT_LIMIT, MOVING};
        }

        namespace rgb_properties{
            static constexpr int stream_width{640}, stream_height{480};
        }

        using Degrees = SI::degree_t<double>;
    }

    class CVNect: public Freenect::FreenectDevice{

        public:
            CVNect(freenect_context *_ctx, int _index): Freenect::FreenectDevice(_ctx, _index), _rgb_buffer(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes){
                // Freenect::FreenectDevice::startVideo();
                // Freenect::FreenectDevice::startDepth();
            }

            virtual ~CVNect(){
                // Freenect::FreenectDevice::stopVideo();
                // Freenect::FreenectDevice::stopDepth();
            }

            void VideoCallback(void* video, [[maybe_unused]] uint32_t timestamp) override{
                // cv::Mat rgb{cv::Size(640 , 480), CV_8UC3, cv::Scalar(0)};
                std::unique_lock lck(_rgb_mtx);
                uint8_t* rgb_data = static_cast<uint8_t*>(video);
                std::copy(rgb_data, rgb_data + this->getVideoBufferSize(), _rgb_buffer.begin());
                cv::Mat rgb(_rgb_buffer, false);
                _video_signal(rgb);
            }

            void DepthCallback(void* _depth, [[maybe_unused]] uint32_t timestamp) override{
            }

            virtual void register_video_signal(const std::function<void (cv::Mat)>& cb){
                _video_signal.connect(cb);
            }

        protected:
            boost::signals2::signal<void (cv::Mat)> _video_signal;
            std::vector<uint8_t> _rgb_buffer{};
            std::mutex _rgb_mtx;

    };

    template<class DeviceType, typename = typename std::enable_if_t<std::is_base_of_v<Freenect::FreenectDevice, DeviceType>>>
    class Kinect;

    template<class DeviceType>
    class Kinect<DeviceType> final{
        public: //properties and structs
            struct State{
                SI::degree_t<double> cmded_angle{};
                units::tilt_properties::status tilt_status{units::tilt_properties::status::STOPPED};
                units::eLEDColors led_color{units::eLEDColors::OFF};
            };

            struct RGBStream{

            };

        public: //members
            const int kID;

            ~Kinect(){
                freenect_shutdown(_fn_cntx);
                _stop_flag.set_value();
                try{
                _dev->stopDepth();
                _dev->stopVideo();
                }catch(...){

                }
            }

            Kinect(int id, std::function<void (cv::Mat)> video_cb): kID(id){
                //init context
                if(freenect_init(&_fn_cntx, NULL) < 0) throw std::runtime_error("Cannot initialize freenect library");
                // We claim both the motor and camera devices, since this class exposes both.
                // It does not support audio, so we do not claim it.
                freenect_select_subdevices(_fn_cntx, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA)); 
                freenect_set_log_level(_fn_cntx, freenect_loglevel::FREENECT_LOG_FATAL);

                int num_devices = freenect_num_devices(_fn_cntx);
                

                //init underlying device
                _dev = std::make_unique<DeviceType>(_fn_cntx, kID);
                if(!_dev || num_devices == 0/* no devices*/){
                    throw std::runtime_error(absl::StrFormat("Couldn't initalize underlying subdevice for Kinect{%i}", kID));
                }

                //cameras
                _dev->register_video_signal(video_cb);
                _dev->startVideo();
                _dev->startDepth();

                _dev_update_thrd = std::jthread(&Kinect::update_task, this);
            }

            State get_state(){
                using namespace std::chrono_literals;
                std::shared_lock slck(_state_mtx);
                return _state;
            }

            void set_tilt(const SI::degree_t<double>& deg){
                _state.cmded_angle = deg;
                _dev->setTiltDegrees(deg.value());
            }

            void set_color(units::eLEDColors color){
                switch(color){
                    case units::eLEDColors::BLINK_GREEN:
                        _dev->setLed(freenect_led_options::LED_BLINK_GREEN);
                        break;
                    case units::eLEDColors::BLINK_R_Y:
                        _dev->setLed(freenect_led_options::LED_BLINK_RED_YELLOW);
                        break;
                    case units::eLEDColors::RED:
                        _dev->setLed(freenect_led_options::LED_RED);
                        break;
                    case units::eLEDColors::GREEN:
                        _dev->setLed(freenect_led_options::LED_GREEN);
                        break;
                    case units::eLEDColors::YELLOW:
                        _dev->setLed(freenect_led_options::LED_YELLOW);
                        break;
                    default:
                        _dev->setLed(freenect_led_options::LED_OFF);
                        break;
                }
                std::unique_lock lck(_state_mtx);
                _state.led_color = color;
            }


            bool operator<(const Kinect& rhs)const{
                return kID < rhs.kID; 
            }

        private: //vars

            //state info
            State _state;
            mutable std::shared_timed_mutex _state_mtx;

            std::promise<void> _stop_flag;
            std::jthread _dev_update_thrd;
            freenect_context* _fn_cntx;
            std::unique_ptr<DeviceType> _dev{nullptr};

        private://funcs

            void update_task(){
                using namespace std::chrono;
                using namespace std::chrono_literals;
                thread_local auto stop = _stop_flag.get_future();
                thread_local auto now = high_resolution_clock::now();
                static constexpr auto delay_time = 100ms;
                thread_local static timeval timeout = {0,1000};
                while(stop.wait_until(now + delay_time) == std::future_status::timeout){
                    now = high_resolution_clock::now();
                    try{
                        _dev->updateState();
                    }catch(...){
                        //TODO: log bad update state
                    }
                    int update_status = freenect_process_events_timeout(_fn_cntx, &timeout);
                    if(update_status < 0){
                        //TODO: log error
                        if (update_status != LIBUSB_ERROR_INTERRUPTED){
                            // This happens sometimes, it means that a system call in libusb was interrupted somehow (perhaps due to a signal)
                            // The simple solution seems to be just ignore it.
                            // std::cerr << "Couldn't update status\n";
                        }
                    }else{
                        std::unique_lock _lck(_state_mtx);
                        auto reported_tilt = _dev->getState().getTiltDegs();
                        if(fabs(_state.cmded_angle.value() - reported_tilt) <= units::tilt_properties::epsilon){
                            _state.cmded_angle.setValue(reported_tilt);
                        }
                    }
                }
            }

    };


}//end cerberus::kinect


#endif //__CERBERUS_KINECT_H_