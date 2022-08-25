#ifndef __CERBERUS_KINECT_H_
#define __CERBERUS_KINECT_H_


// #include <libfreenect.h>
#include <libfreenect/libfreenect.hpp>
#include <absl/strings/str_format.h>

#include <SI/mass.h>
#include <SI/area.h>
#include <SI/angle.h>

#include <functional>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <future>
#include <chrono>
#include <mutex>





namespace cerberus::kinect{


    const SI::kilo_gram_t<long double> kg{0};
    namespace units{
        enum class eLEDColors{OFF = 0, GREEN, RED, YELLOW, BLINK_GREEN, BLINK_R_Y};
        struct TiltProperties{
            static constexpr double fast_step = 3, slow_step = 1;
            static constexpr double epsilon{0.1};
            enum class status{STOPPED, AT_LIMIT, MOVING};
        };
    }

    class Kinect final{
        public: //properties and structs
            struct State{
                SI::degree_t<double> cmded_angle{};
                units::TiltProperties::status tilt_status{units::TiltProperties::status::STOPPED};
            };
        public: //members
            const int kID;

            ~Kinect(){
                _stop_flag.set_value();
            }

            explicit Kinect(int id = 0): kID(id){
                //init context
                if(freenect_init(&_fn_cntx, NULL) < 0) throw std::runtime_error("Cannot initialize freenect library");
                // We claim both the motor and camera devices, since this class exposes both.
                // It does not support audio, so we do not claim it.
                freenect_select_subdevices(_fn_cntx, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA)); 
                //init underlying device
                _dev = std::make_unique<Freenect::FreenectDevice>(_fn_cntx, kID);
                if(!_dev){
                    throw std::runtime_error(absl::StrFormat("Couldn't initalize underlying subdevice for Kinect{%i}", kID));
                }
                _dev_update_thrd = std::jthread(&Kinect::update_task, this);
            }

            std::optional<State> get_state(){
                using namespace std::chrono_literals;
                std::unique_lock lck(_state_mtx, std::defer_lock);
                if(lck.try_lock_for(5ms)){
                    std::make_optional<State>(_state);
                }
                return std::nullopt;
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
            }


            bool operator<(const Kinect& rhs)const{
                return kID < rhs.kID; 
            }

        private: //vars

            //state info
            State _state;
            std::timed_mutex _state_mtx;

            std::promise<void> _stop_flag;
            std::jthread _dev_update_thrd;
            freenect_context* _fn_cntx;
            std::unique_ptr<Freenect::FreenectDevice> _dev{nullptr};

        private://funcs

            void update_task(){
                using namespace std::chrono;
                using namespace std::chrono_literals;
                thread_local auto stop = _stop_flag.get_future();
                thread_local auto now = high_resolution_clock::now();
                static constexpr auto delay_time = 100ms;
                thread_local static timeval timeout = {0, 50 };
                while(stop.wait_until(now + delay_time) == std::future_status::timeout){
                    now = high_resolution_clock::now();
                    _dev->updateState();
                    int update_status = freenect_process_events_timeout(_fn_cntx, &timeout);
                    if(update_status < 0){
                        //TODO: log error
                        std::cerr << "Couldn't update status\n";
                    }
                }
            }

    };


}//end cerberus::kinect


#endif //__CERBERUS_KINECT_H_