#ifndef __CERBERUS_KINECT_H_
#define __CERBERUS_KINECT_H_

#include "libfreenect_sync.h"
#include <functional>
#include <libfreenect.h>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <future>
#include <chrono>


#include <absl/strings/str_format.h>


namespace cerberus::kinect{



    namespace units{
        enum class eLEDColors{OFF = 0, GREEN, RED, YELLOW, BLINK_GREEN, BLINK_R_Y};
    }

    //TODO: define a runtime & install config 
    class Kinect{
        public://vars
            enum eModels{V1, V2};
            const int8_t kID;
            const eModels kModel;

        public://funcs
            ~Kinect(){
                _quit_signal.set_value();
                // while(_update_thread.joinable()){
                //     std::this_thread::sleep_for(std::chrono::seconds(1));
                // }
            }

            explicit Kinect(int8_t kid, eModels model = eModels::V1): kID(kid), kModel(model){
                _update_thread = std::jthread(&Kinect::_update_thread_routine, this);
            }

            void set_tilt(int deg){
                _tilt_motor.set_deg(deg, kID);
            }

            void set_color(units::eLEDColors color){
                _led_dev.send_color(color, kID);
            }

            units::eLEDColors color(){
                return _led_dev.color;
            }
            

            std::optional<freenect_raw_tilt_state> tilt_state() const{
                std::unique_lock lck(_tilt_motor.locked_state.mtx);
                if(_tilt_motor.locked_state.state){
                    return std::make_optional(*_tilt_motor.locked_state.state.get());
                }
                return std::nullopt;
            }

            bool operator<(const Kinect& rhs)const{
                return kID < rhs.kID; 
            }

        private: //vars
            std::promise<void> _quit_signal;
            std::jthread _update_thread;
            
            struct TiltMotor{

                struct LockedState{
                    mutable std::mutex mtx;
                    std::unique_ptr<freenect_raw_tilt_state> state = std::make_unique<freenect_raw_tilt_state>();
                }locked_state;

                double x_ax{0.}, y_ax{0.}, z_ax{0.};
                int16_t usr_ang_deg{0};
                int8_t kID{0};

                void set_deg(int degrees, int8_t id){
                    if(freenect_sync_set_tilt_degs(degrees, id)){
                        //TODO: log error
                    }
                }

                int update_state(){//update the raw values from the kinect
                    std::unique_lock _lck(locked_state.mtx);
                    auto* state_ptr = locked_state.state.get();
                    int result = freenect_sync_get_tilt_state(&state_ptr, kID);
                    //update the accelerometer data
                    if(result >=0 && locked_state.state){
                        freenect_get_mks_accel(locked_state.state.get(), &x_ax, &y_ax, &z_ax);
                    }
                    return result;
                }

            } _tilt_motor;

            struct LED_DEV{
                units::eLEDColors color{units::eLEDColors::OFF};
                int8_t kID{0};
                void send_color(units::eLEDColors new_color, int ID){
                    color = new_color;
                    switch(new_color){
                        case units::eLEDColors::BLINK_GREEN:
                            freenect_sync_set_led(freenect_led_options::LED_BLINK_GREEN, ID);
                            break;
                        case units::eLEDColors::BLINK_R_Y:
                            freenect_sync_set_led(freenect_led_options::LED_BLINK_RED_YELLOW, ID);
                            break;
                        case units::eLEDColors::RED:
                            freenect_sync_set_led(freenect_led_options::LED_RED, ID);
                            break;
                        case units::eLEDColors::GREEN:
                            freenect_sync_set_led(freenect_led_options::LED_GREEN, ID);
                            break;
                        case units::eLEDColors::YELLOW:
                            freenect_sync_set_led(freenect_led_options::LED_YELLOW, ID);
                            break;
                        default:
                            freenect_sync_set_led(freenect_led_options::LED_OFF, ID);
                            break;
                    }
                }
            }_led_dev;

        private: //funcs

            void _update_thread_routine(){
                auto quit = _quit_signal.get_future();
                using namespace std::chrono_literals;
                using namespace std::chrono;
                auto current_time = std::chrono::high_resolution_clock::now();
                static constexpr auto ms_between_loops = 0.1s;
                _tilt_motor.kID = kID;
                while(quit.wait_until(current_time + ms_between_loops) != std::future_status::ready){
                    current_time = high_resolution_clock::now();
                    int get_result{0};
                    
                    _tilt_motor.update_state();
                    if(get_result < 0){
                        //TODO: log failed to update state
                        std::cerr << absl::StrFormat("Failed to update Tilt Motor State for Kinect %i\n", kID);
                    }
                    
                }
            }
    };


}//end cerberus::kinect


#endif //__CERBERUS_KINECT_H_