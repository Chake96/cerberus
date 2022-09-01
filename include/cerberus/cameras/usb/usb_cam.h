#ifndef __CERBERUS_CAMERAS_USB_CAM_H_
#define __CERBERUS_CAMERAS_USB_CAM_H_

#include <string_view>

namespace cerberus::cameras::usb {

    class USBCamera {
      public:  //methods
               // USBCamera() {}
      private: //vars
        const std::string_view _usb_descriptor;
    };

} // namespace cerberus::cameras::usb

#endif //__CERBERUS_CAMERAS_USB_CAM_H_